# First time

Add the right repository form [here](https://launchpad.net/~beineri)


    sudo add-apt-repository ppa:beineri/opt-qt512-xenial -y

Then, run:

    sudo apt-get update
    sudo apt-get install qt512base qt512svg qt512declarative-y

Download the latest version of [LinuxDeployQt](https://github.com/probonopd/linuxdeployqt) and make it executable with __chmod__:

    wget "https://github.com/probonopd/linuxdeployqt/releases/download/continuous/linuxdeployqt-continuous-x86_64.AppImage" -O ~/linuxdeployq.AppImage
    chmod +x ~/linuxdeployq.AppImage
    
    sudo apt-get -y install libgtk2.0-dev
    git clone http://code.qt.io/qt/qtstyleplugins.git
    cd qtstyleplugins
    source /opt/qt512/bin/qt512-env.sh
    /opt/qt512/bin/qmake
    make -j$(nproc)
    sudo make install 
    cd ..
    rm -rf qtstyleplugins

# Build the AppImage with catkin_make

In the root folder of ws_plotjuggler:

    rm -rf buil devel install
    source /opt/qt512/bin/qt512-env.sh
    unset QTDIR; unset QT_PLUGIN_PATH ; unset LD_LIBRARY_PATH
    export LD_LIBRARY_PATH=/opt/ros/kinetic/lib
    catkin_make -DCMAKE_BUILD_TYPE=Release  -j$(nproc) install  

    export VERSION=2.1.1 
    ~/linuxdeployq.AppImage ./install/lib/plotjuggler/PlotJuggler.desktop  -appimage -appimage -extra-plugins=iconengines,imageformats -extra-plugins=platformthemes/libqgtk2.so,styles/libqgtk2style.so



