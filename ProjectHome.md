Intrael is a server that provides an HTTP interface for the MS kinect. It processes the depth stream coming from the device and thresholds it based on fixed depth ranges or a reference background frame. It then measures several properties for the blobs it finds and provides them to network clients wrapped as JSON arrays. These can be retrieved through polling with XHRs or real streaming with Server Sent Events.

Using nothing more than plain AJAX, computer vision can be performed directly in the browser. From smart surveillance cameras to all kinds of interactive surfaces, a multitude of new possibilities opens up for web development. The client server approach allows great versatility.

The data provided for each blob consist of 3D points for the extremes of the x,y,z axes plus the geometric center of the object, along with depth readings of the corresponding points on the (thresholded) background. Finally the pack includes blob pixel counts and more general info.

The raw outputs from the cameras are also provided as either JPEG images, MJPEG streams or uncompressed binary frames. The last form can be used with JS typed arrays or whatever else is provided by the language of your choice and can be very useful for further processing.

The server works great even on low powered platforms like the beagleboard making it ideal for embedded devices. It uses libfreenect for the kinect part which means that you can use the xbox kinect to build cost effective solutions. The code is licenced as free software(**AGPLv3**).

**After starting it go to http://test.intrael.com. If the browser runs on a different box enter http://{server_address}:6661 on the input field**

**A port of intrael to the MS Kinect SDK is available [here](https://code.google.com/p/kinect-server-strokeback/). The porting was done under contract from RFSAT for the needs of the EU funded strokeback project**

**Version 2.0 RC1 Released** in the SVN repository. This is a major revision that brings lots of enhancements but doesn't change the protocol. Consult the docs that come with the source and the examples at the JSDo.it site to check out how it works. It's much more dev friendly now. **Linux only for now, platform fixes incoming** :)

**Version 1.1 released** with various fixes, capabilities checking and Server Sent Events. This last one allows for true streaming of the JSON data and you can enable it by providing a -m 7 on the startup command line flags. All the examples have been updated to allow for transparent handling of the method selected. Unfortunately as no browser allows cross origin SSE yet(patches are in though) you'll have to disable CORS checking on your browser to use it. For chrome you'll need the command line flag --disable-web-security.

| <a href='http://www.youtube.com/watch?feature=player_embedded&v=sIXIGeJSuAg' target='_blank'><img src='http://img.youtube.com/vi/sIXIGeJSuAg/0.jpg' width='425' height=344 /></a> | <a href='http://www.youtube.com/watch?feature=player_embedded&v=uXRKgvox1xQ' target='_blank'><img src='http://img.youtube.com/vi/uXRKgvox1xQ/0.jpg' width='425' height=344 /></a> |
|:----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|:----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| <a href='http://www.youtube.com/watch?feature=player_embedded&v=fBNN8cKFla4' target='_blank'><img src='http://img.youtube.com/vi/fBNN8cKFla4/0.jpg' width='425' height=344 /></a> | <a href='http://www.youtube.com/watch?feature=player_embedded&v=oIs8E0mgIOc' target='_blank'><img src='http://img.youtube.com/vi/oIs8E0mgIOc/0.jpg' width='425' height=344 /></a> |