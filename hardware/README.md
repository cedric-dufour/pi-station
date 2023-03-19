Raspberry Pi Station - Hardware
===============================

Base Components
---------------

The following are required as base components:

* [Raspberry Pi 3 B+][pi]

* heatsink for the Raspberry Pi SoC

* industrial-grade [Kingston SDCIT2][sd] SD card

* [Sleepy Pi][pm] power management controller (Raspberry Pi hat extension)

* short and "thick-enough" USB cables (lower resistance-induced losses)

* a fan to maintain a constant cooling airflow within the enclosure

[pi]: ./ref/RaspberryPi%203Bp%20-%20datasheet.pdf
[pm]: ./ref/Sleepy%20Pi%202%20(power%20management)%20-%20spellfoundry.pdf
[sd]: ./ref/Kingston%20SDCIT2%20(SD%20card)%20-%20datasheet.pdf
[hub]: ./ref/EX-1197HMS%20(USB%203%20hub)%20-%20datasheet.pdf

Leveraging the Sleepy Pi offers several benefits:

* it is friendly to 12V or 24V power sources, e.g. solar panels

And allows:

* to gracefully shut down and power on the Raspberry Pi

* monitor supply voltage and drawn current (using the included Arduino)

* control the fan power along the Raspberry Pi or enclosure temperature

* lower global power consumption (e.g. along solar power sources)


### Demonstrated use-case (FLARM receiver)

To enable the station as a FLARM receiver, one shall add:

* [NooElec NESDR SMArt][sdr] (Software-Definer Radio) USB dongle

* [Huawei E3372][gsm] LTE USB dongle (optionally, if no WiFi is available)

[sdr]: ./ref/Nooelec%20NESDR%20SMArt%20v4%20(SDR%20dongle)%20-%20datasheet.pdf
[gsm]: ./ref/Huawei%20E3372%20(LTE%20dongle)%20-%20datasheet.pdf


AC-powered version
------------------

To power the entire apparatus from the facility (110-240V AC), one needs to add:

* [MeanWell 12V 36W][psu] power supply

* "small-factor" [Legrand][encl] 290x190x90mm enclosure

[psu]: ./ref/MeanWell%20LRS-35-12%20(power%20supply)%20-%20datasheet.pdf
[encl]: ./ref/Legrand%20(enclosures)%20-%20datasheet.pdf

Which can be assembled as per given [layout](./layout.pdf).


Solar-powered version
---------------------

To use solar power - and make the station entirely independent - one needs to add:

* [Phaesun SunPlus][sp] 20W solar panel

* [IVT Seriell][sc] 12V/4A solar charger (incl. deep discharge protection)

* [Sonnenschein Battery][bat], recommendly 12V/12Ah along Power Management option
  or larger for 24/24h use cases

* "large-factor" [Legrand][encl] enclosure (with battery inside)

[sp]: ./ref/Phaesun%20SunPlus%20(solar%20panels)%20-%20datasheet.pdf
[sc]: ./ref/IVT%20Solartechnik%20(solar%20chargers)%20-%20brochure.pdf
[bat]: ./ref/Sonnenschein%20A400%20(batteries)%20-%20datasheet.pdf
