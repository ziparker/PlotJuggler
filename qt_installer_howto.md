# Download QT Installer Framework

https://download.qt.io/official_releases/qt-installer-framework/

# Create the installer

mkdir build
cd build
cmake ..
make -j$(nproc)
cd ..

cp build/bin* installer/io/plotjuggler.app/data/

~/Qt/QtIFW-3.0.6/bin/binarycreator --offline-only -c installer/config.xml -p installer  PlotJugglerInstaller
