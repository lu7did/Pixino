## Ham radio Pixie Transceiver controlled by an Ardunio Nano board

The Pixino board is specialized on digital modes and inspired on the algorithms
created by Summers (G0UPL) but with a more primitive implementation for
experimentation uses only, if you plan to operate seriously on digital modes
buy the excellent QRP-Labs QDX kit.

This is a project built on top of the uSDX firmware (QCX-SSB) by Guido (PE1NNZ)
with modifications from Pablo (EA2EHC).

# Intended Features

Initial project setup for HF Arduino controlled Pixie transceiver:

*     Operate CW-USB
*     LCD 16x2 + encoder based control
*     Dual VFO (A/B) for split mode
*     VOX & PTT control
*     TS-480 CAT command
*     Operates 7.0-7.3 MHz
*     Largely inspired on Guido (PE1NNZ) with QCX-SSB.
*     Largely based on my own projects Pi4D/PixiePi

# Background

The Pixie QRPp (very low power, less than 1W output) transceiver is a very popular DIY project among hams as it is
very easy to build, test and operate from the electronic standpoint, yet able to perform some actual limited communications
(QSO) over the air.

Although really small power can be enough to sustain communications over great distances and therefore not being a per se
limiting factor there are other factors in the basic implementation which makes difficult to carry communications except
on very favorable conditions.

An explanation of how the transceiver work can be found [here](http://w1sye.org/wp-content/uploads/2017/01/NCRC_PixieOperation.pdf).

This project starts with a working Pixie transceiver (a cheap kit bought at eBay or other sellers) and to integrate it with
an Arduino Uno board controlling a Si 5351 board to provide the signal generation and other functionality.

Instead of a crystal based signal generation DDS is be used.

The rest of the code deals mostly with the user interface and operating features, among others:

## DISCLAIMER

This is a pure, non-for-profit, project being performed in the pure ham radio spirit of experimentation, learning and sharing.
This project is original in conception and has a significant amount of code developed by me, it does also being built up on top
of a giant effort of others to develop the base platform and libraries used.
Therefore this code should not be used outside the limits of the license provided and in particular for uses other than
ham radio or similar experimental purposes.
No fit for any purpose is claimed nor guaranteed, use it at your own risk. The elements used are very common and safe, the skills
required very basic and limited, but again, use it at your own risk.
Despite being a software enginering professional with access to technology, infrastructure and computing facilities of different sorts
I declare this project has been performed on my own time and equipment.
This project must not be seen as a replacement for other fine kits such as the QCX, D4D, QRPGuys and similar; after all it's based on
just a USD 4.- Pixie board, so it can be compared and used with other similar boards as well as long as the Xtal can be replaced by a
DDS.

# Background

This project has been largely inspired by the work from [Guido (PE1NNZ)](http://pe1nnz.nl.eu.org) and in particular
by the [QCX-SSB](https://github.com/threeme3/QCX-SSB) project and the entire echosystem around it.

Also my own work with the projects [PixiePi](http://www.github.com/lu7did/PixiePi) and [OrangeThunder](http://www.github.com/lu7did/OrangeThunder) had
been largely used to extract code for this project.


