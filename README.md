# CSTBox extension for Carlo Gavazzi ModBus products support

This repository contains the code for the extension adding the support
for Carlo Gavazzi ModBus based products in the [CSTBox framework](http://cstbox.cstb.fr). 

Carlo Gavazzi products are industrial modules for electrical measures. More details can be found
on their [Web site](http://www.gavazziautomation.com/nsc/hq/en/).

The support comes in two forms :

  * product drivers generating CSTBox events from registers map readings
  * products definition files (aka metadata) driving the associated Web configuration editor
    pages

## Currently supported products

  * **EM21**
      * tri-phases AC multi-measures module
      * outputs : U, I, P, Q, F, power factor, active W, reactive W

## Runtime dependencies

This extension requires the CSTBox core and ModBus support extension to be already installed.
