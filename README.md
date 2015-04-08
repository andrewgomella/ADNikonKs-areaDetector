ADNikonKs
===========
An 
[EPICS](http://www.aps.anl.gov/epics/) 
[areaDetector](http://cars.uchicago.edu/software/epics/areaDetector.html) 
driver for the Nikon Qi2 camera using KsCamera SDK. [DS-Qi2](http://www.nikon.com/products/instruments/lineup/bioscience/camera_microscopy/dsqi2/index.htm)

Originally built with the following setup:
	
	Compiling Software:
		Windows 8 Pro x64
		Microsoft Visual Studio 2013 (12) with update 4
		ActivePerl 5.16.3 Build 1603 (64-bit)
		GnuWin32:Make-3.81
		GnuWin32:Re2C version 0.9.4

	EPICS setup:
		base-3.14.12.4
		 -with SHARED_LIBRARIES=NO
		 -with STATIC_BUILD=YES

		support
			areaDetector-R2-0
				-precompiled binaries for Windows downloaded from github
			asyn-4-26
			autosave-5-1
			busy-1-6
			calc-3-2
			seq-2-1-13
			sscan-2-9

	Nikon SDK:
		Version 1.0.0.4
		downloaded from [nisdk.net](http://www.nisdk.net/)
		(requires authorization to join and download files are password protected)

Before compiling run "C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat x86_amd64" or equivalent.

To send arrays over EPICS channel access make sure EPICS_CA_MAX_ARRAY_BYTES is set to an adequate amount. 