# First time

Add the right repository form [here](https://launchpad.net/~beineri)


    sudo add-apt-repository ppa:beineri/opt-qt-5.12.3-xenial -y

Then, run:

    sudo apt-get update
    sudo apt-get install qt512base qt512svg qt512declarative qt512translations qt512multimedia -y

Download the latest version of [LinuxDeployQt](https://github.com/probonopd/linuxdeployqt) and make it executable with __chmod__:

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

    wget https://github.com/linuxdeploy/linuxdeploy/releases/download/continuous/linuxdeploy-x86_64.AppImage
    chmod +x linuxdeploy*.AppImage

    rm -rf build devel install
    source /opt/qt512/bin/qt512-env.sh
    catkin_make -DCMAKE_BUILD_TYPE=Release  -j$(nproc) install  
    
    cd src/PlotJuggler;export VERSION=$(git describe --abbrev=0 --tags);cd -;echo $VERSION
    
    mkdir -p AppDir/usr/bin/
    cp install/lib/plotjuggler/* AppDir/usr/bin
    
    ./linuxdeploy-x86_64.AppImage --appdir=AppDir -d src/PlotJuggler/PlotJuggler.desktop -i src/PlotJuggler/plotjuggler-icon.png --output appimage
     
    


