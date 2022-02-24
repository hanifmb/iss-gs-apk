# iss-gs-apk

ISS-GS-APK is a guest science program that was developed for the KIBO RPC's final round in the ISS. The program runs a mission that contains a predefined navigation route, a series of QR decoding, and a fiducial marker pose estimation. For comprehensive information on the mission, please refer to the [official Guidebook](https://humans-in-space.jaxa.jp/krpc/1st/download/index.html). Referring to 1.7.3 in the Guidebook, this program is set to the public only for educational purposes.

# Installation and Running the simulation

The difference between this version and the [preliminary round](https://github.com/villainjoe/sim-gs-apk) version are the updated API for flashlight functionality and the newly designed mission. Moreover, this version is equipped with the additional mission to run in a real-world environment by considering the most straightforward approach and real-world camera parameters.

Note that this version of the guest science program might not be compatible with the current version of the [Astrobee simulator](https://github.com/nasa/astrobee). The exact version of the simulation environment is needed to guarantee that the simulation will run as expected. Please refer to the KIBO-RPC 2020 [programming manual](https://humans-in-space.jaxa.jp/krpc/1st/download/index.html) for the simulation installation and software versions.

In the case that the program is run for the simulation, the camera parameter needs to be set for the simulated camera by swapping the `initCamera(Param.ORBIT);` for the `initCamera(Param.SIMULATION);` in the main program: [https://github.com/villainjoe/iss-gs-apk/blob/525965628b8ade690acb9c23e51d9589336d7b00/app/src/main/java/jp/jaxa/iss/kibo/rpc/indonesia/YourService.java#L141](https://github.com/villainjoe/iss-gs-apk/blob/525965628b8ade690acb9c23e51d9589336d7b00/app/src/main/java/jp/jaxa/iss/kibo/rpc/indonesia/YourService.java#L141)

For building the APK and installation at the simulated HLP please refer to the preliminary round version under the **Another Bash Commands**. The guide to running the simulation can also be found on the same repository. Please be aware that all the steps needed from building the APK to run the simulation can be found in the aforementioned official Guidebook and programming manual.

# Astrobee related documentation

[Localization from Visual Landmarks on a Free-flying Robot](https://www.nasa.gov/sites/default/files/atoms/files/coltin2016localization.pdf)

[Astrobee Robot Software: Enabling Mobile Autonomy on the ISS](https://www.nasa.gov/sites/default/files/atoms/files/fluckiger2018astrobee.pdf)

[Astrobee Guest Science Guide](https://www.nasa.gov/sites/default/files/atoms/files/irg-ff029-astrobee-guest-science-guide.pdf)

[ASTROBEE: A NEW PLATFORM FOR FREE-FLYING ROBOTICS ON THE INTERNATIONAL SPACE STATION](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20160007769.pdf)

[Astrobee: A New Platform for Free-Flying Robotics on the ISS (Presentation)](http://longhorizon.org/trey/papers/smith16_astrobee_design_slides.pdf)
