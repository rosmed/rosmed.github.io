If you are planning to participate in our workshop, we recommend to download/install the following package prior to the workshop, as it may take some time to download them. 

3D Slicer
=========

In this tutorial, we will use 3D Slicer version 4.8.1 Binaries are available from the following links:
- [Slicer 4.8.1 for Linux (Intel 64-bit)](http://slicer.kitware.com/midas3/download/item/330417/Slicer-4.8.1-linux-amd64.tar.gz)
- [Slicer 4.8.1 for macOS (Intel 64-bit)](http://slicer.kitware.com/midas3/download/item/330418/Slicer-4.8.1-macosx-amd64.dmg)
- [Slicer 4.8.1 for Windows (Intel 64-bit)](http://slicer.kitware.com/midas3/download/item/329467/Slicer-4.8.1-win-amd64.exe)

**Note for Mac users:** If the system pops up a window warning that "Slicer.app can't be opened because it is from an unidentified developer" when 3D Slicer is launched for the first time, please start the Slicer application by clicking the icon with right mouse button (or click with a Ctrl key) and select "Open" from the pull down menu. Then you will be prompted to confirm that you are opening the application. Slicer will be launched once "Open" button is clicked.

We will not use the latest version of 3D Slicer (4.10.1 as of March 2019) in this tutorial, as it still has a few issues with transferring points and polydata to and from ROS-IGTL-Bridge.  

After installing and launching 3D Slicer, open the Extension Manager ("View" -> "Extension Manager"), and install the following extension:

- **SlicerIGT**

After restarting the 3D Slicer, you should see plug-in modules included in the extension under "IGT" section of the modules menu.




