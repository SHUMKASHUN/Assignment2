# For Windows, refer to the [Wiki](https://github.com/robomasterhkust/RMTutorialF103/wiki) pages

# For Unix system(Tested on Ubuntu 16.04 & Mac OS Sierra 10.12.6)
By Yang Shaohui, Alex Au on 2018.09.13

## Getting started on Unix
run the following commands in the project directory
1.  `sudo apt-get install openocd gcc-arm-none-eabi` under linux. 

    `brew install open-ocd`
    `brew tap PX4/px4`
    `brew install px4-dev` under OSX. 
    `brew cask install xquartz java` might be needed. Also, the XCode command line toolchain is needed as dependency. If you do not have homebrew installed yet, go to https://brew.sh. 

2. `git clone` and `cd` into this repository

4. `make` or `make -j4` where 4 is the thread count

5. run `openocd -f openocd/start_st-link_flash.cfg` with board connected via st-link to flash the program
---
#### Optional:
- for debugging, run `openocd -f openocd/start_st-link_debug.cfg` , followed by a gdb connection with you own IDE/editor, the VSCode configuration is provided and is used with the "Native Debug" extension
- of course you can always use `telnet localhost 4444` and use terminal to interface with the openocd debug server directly

## Submitting assignments 
This part will come up later on. 
