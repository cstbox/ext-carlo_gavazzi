#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This file is part of CSTBox.
#
# CSTBox is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# CSTBox is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with CSTBox.  If not, see <http://www.gnu.org/licenses/>.

""" Carlo Gavazzi 3-phased EM21 lew-level interface.

This modules defines a sub-class of minimalmodbus.Instrument which polls the
parameters of interest.

It also defines the input registers so that more specific needs can be
implemented by sub-classing and overiding, or direct Modbus register read.

Depends on Jonas Berg's minimalmodbus Python library :
    https://pypi.python.org/pypi/MinimalModbus/
    Version in date of writing: 0.4
"""

import minimalmodbus
import struct
from collections import namedtuple
import time
import logging

from pycstbox.modbus import ModbusRegister
from pycstbox.log import Loggable

__author__ = 'Eric PASCUAL - CSTB (eric.pascual@cstb.fr)'


class EM21Instrument(minimalmodbus.Instrument, Loggable):
    """ minimalmodbus.Instrument sub-class modeling the Carlo Gavazzi EM21
    3-phased energy meter.

    The supported model is the RTU RS485 one, the RS485 bus being connected
    via a USB.RS485 interface.
    """

    DEFAULT_BAUDRATE = 9600

    class EM21_INT32Reg(ModbusRegister):
        def __new__(cls, addr, *args, **kwargs):
            """ Overridden __new__ for fixing the register size and
            forcing unsigned values since 2's complement is used here.
            """
            return ModbusRegister.__new__(cls, addr, size=2, signed=False, *args, **kwargs)

        @staticmethod
        def decode(raw):
            # swap the words and apply the 2's complement if negative value
            # BTW discard the signed since negative values have no real meaning
            raw = ((raw >> 16) & 0xffff) | ((raw & 0xffff) << 16)
            return abs(raw - 0x100000000) if raw & 0x80000000 else raw

    class VoltageRegister(EM21_INT32Reg):
        @staticmethod
        def decode(raw):
            return EM21Instrument.EM21_INT32Reg.decode(raw) / 10.

    class CurrentRegister(EM21_INT32Reg):
        @staticmethod
        def decode(raw):
            return EM21Instrument.EM21_INT32Reg.decode(raw) / 1000.

    class PowerRegister(EM21_INT32Reg):
        @staticmethod
        def decode(raw):
            return EM21Instrument.EM21_INT32Reg.decode(raw) / 10.

    class PowerFactorRegister(ModbusRegister):
        def __new__(cls, addr, *args, **kwargs):
            """ Overridden __new__ for fixing the register size. """
            return ModbusRegister.__new__(cls, addr, *args, **kwargs)

        @staticmethod
        def decode(raw):
            return raw / 1000.

    class FrequencyRegister(ModbusRegister):
        def __new__(cls, addr, *args, **kwargs):
            """ Overridden __new__ for fixing the register size. """
            return ModbusRegister.__new__(cls, addr, *args, **kwargs)

    class EnergyRegister(EM21_INT32Reg):
        @staticmethod
        def decode(raw):
            return EM21Instrument.EM21_INT32Reg.decode(raw) / 10.

    V_L1_N = VoltageRegister(0x00)
    V_L2_N = VoltageRegister(0x02)
    V_L3_N = VoltageRegister(0x04)
    V_L1_L2 = VoltageRegister(0x07)
    V_L2_L3 = VoltageRegister(0x08)
    V_L3_L1 = VoltageRegister(0x0A)
    A_L1 = CurrentRegister(0x0C)
    A_L2 = CurrentRegister(0x0E)
    A_L3 = CurrentRegister(0x10)
    W_L1 = PowerRegister(0x12)
    W_L2 = PowerRegister(0x14)
    W_L3 = PowerRegister(0x16)
    VA_L1 = PowerRegister(0x18)
    VA_L2 = PowerRegister(0x1A)
    VA_L3 = PowerRegister(0x1C)
    VAR_L1 = PowerRegister(0x1E)
    VAR_L2 = PowerRegister(0x20)
    VAR_L3 = PowerRegister(0x21)
    V_L_N_SYS = VoltageRegister(0x24)
    V_L_L_SYS = VoltageRegister(0x026)
    W_SYS = PowerRegister(0x028)
    VA_SYS = PowerRegister(0x02A)
    VAR_SYS = PowerRegister(0x02C)
    PF_L1 = PowerFactorRegister(0x02E)
    PF_L2 = PowerFactorRegister(0x02F)
    PF_L3 = PowerFactorRegister(0x030)
    PF_SYS = PowerFactorRegister(0x031)
    FREQ = FrequencyRegister(0x033)
    KWH_SYS = EnergyRegister(0x034)
    KVARH_SYS = EnergyRegister(0x036)

    class RegisterBank(object):
        """ A register bank defines a group of registers which can be read in a single request,
        taking in account the EM21 limitation to 11 simple registers per access.
        """
        def __init__(self, *regs):
            self.regs = regs
            self.unpack_format = '>' + ''.join(r.unpack_format for r in regs)
            self.size = reduce(lambda sztot, sz: sztot + sz, [r.size for r in regs])
            if self.size > 11:
                raise ValueError('11 simple regs limit exceeded')

    #: The register banks
    BANKS = [
        RegisterBank(
            V_L1_N, V_L2_N, V_L3_N, V_L1_L2, V_L2_L3
        ),
        RegisterBank(
            V_L3_L1, A_L1, A_L2, A_L3, W_L1
        ),
        RegisterBank(
            W_L2, W_L3, VA_L1, VA_L2, VA_L3
        ),
        RegisterBank(
            VAR_L1, VAR_L2, VAR_L3, V_L_N_SYS, V_L_L_SYS
        ),
        RegisterBank(
            W_SYS, VA_SYS, VAR_SYS,
        ),
        RegisterBank(
            PF_L1, PF_L2, PF_L3, PF_SYS
        ),
        # CAUTION : since we skip the phase sequence register, we need to start a new bank
        # No doing this would include the Ph_seq in the data, thus shifting the content
        # of the fields when decoding the packet
        RegisterBank(
            FREQ, KWH_SYS, KVARH_SYS
        )
    ]

    BANKS_COUNT = len(BANKS)
    LAST_BANK_NUM = BANKS_COUNT - 1

    #: the compiled sequence of banks registers
    ALL_REGS = reduce(lambda accum, v: accum + list(v.regs), BANKS, [])
    
    #: Definition of the type of the poll() method result
    #: .. warning::
    #:      the field sequence MUST be synchronized with the register sequence as defined in `ALL_REGS`
    OutputValues = namedtuple('OutputValues', [
        "V_L1_N", "V_L2_N", "V_L3_N", "V_L1_L2", "V_L2_L3", "V_L3_L1",
        "A_L1", "A_L2", "A_L3",
        "W_L1", "W_L2", "W_L3", "VA_L1", "VA_L2", "VA_L3", "VAR_L1", "VAR_L2", "VAR_L3",
        "V_L_N_sys", "V_L_L_sys", "W_sys", "VA_sys", "VAR_sys",
        "PF_L1", "PF_L2", "PF_L3", "PF_sys",
        "F", "kWh_sys", "kVARh_sys"
    ])

    def __init__(self, port, unit_id, baudrate=DEFAULT_BAUDRATE):
        """
        :param str port: serial port on which the RS485 interface is connected
        :param int unit_id: the address of the device
        :param int baudrate: the serial communication baudrate
        """
        super(EM21Instrument, self).__init__(port=port, slaveaddress=int(unit_id))
        self.serial.close()
        self.serial.setBaudrate(baudrate)
        self.serial.setTimeout(2)
        self.serial.open()
        self.serial.flush()

        self._first_poll = True
        self.poll_req_interval = 0
        self.terminate = False
        self.communication_error = False

        Loggable.__init__(self, logname='em21-%03d' % self.address)

    @property
    def unit_id(self):
        """ The id of the device """
        return self.address

    def poll(self):
        """ Reads all the measurement registers and the values as a named tuple.

        :rtype: OutputValues
        """
        if self._logger.isEnabledFor(logging.DEBUG):
            self._logger.debug("polling %s(%d)", self.__class__.__name__, self.unit_id)

        # read all the registers
        raw_values = []
        for i, bank in enumerate(self.BANKS):
            if self.terminate:
                return None

            if self._logger.isEnabledFor(logging.DEBUG):
                self._logger.debug("reading bank #%d (addr=0x%04x reg_count=%d)", i, bank.regs[0].addr, bank.size)

            # ensure no junk is lurking there
            self.serial.flush()

            try:
                data = self.read_string(bank.regs[0].addr, bank.size)
            except ValueError:
                # CRC error is reported as ValueError
                # => reset the serial link, wait a bit abd abandon this transaction
                self.log_error('trying to recover from error')
                self.serial.close()
                time.sleep(self.poll_req_interval)
                self.serial.open()
                self.communication_error = True
                return None

            if self.communication_error:
                self.log_info('recovered from error')
                self.communication_error = False

            raw = struct.unpack(bank.unpack_format, data)
            if self._logger.isEnabledFor(logging.DEBUG):
                self._logger.debug("... raw: %s", ' '.join(hex(r) for r in raw))
            raw_values.extend(raw)

            # if a pause between low level request must be done, wait a bit before going ahead
            # (apart if this is the last request of the batch, since the ModBus controller
            # does a pause between successive devices)
            if i < self.LAST_BANK_NUM and self.poll_req_interval:
                if self._logger.isEnabledFor(logging.DEBUG):
                    self._logger.debug("pausing %.1fs between ModBus requests...", self.poll_req_interval)
                time.sleep(self.poll_req_interval)

        # decode the raw values
        # (the raw values list is in the same sequences as the registers,
        # which enumeration description is synchronized with the OutputValues definition)
        values = self.OutputValues(*[r.decode(v) for v, r in zip(raw_values, self.ALL_REGS)])
        if self._logger.isEnabledFor(logging.DEBUG):
            self._logger.debug("==> %s", values)
        return values
