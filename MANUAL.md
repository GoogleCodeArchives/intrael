<h2>NAME</h2>

intrael - Computer vision for the web
<p>

<i></i>
<b>intrael</b> <a href='.md'>options </a>

</p><p>
<a> </a>
</p><h2>DESCRIPTION</h2>

The <b>intrael</b> server processes the depth stream from the Kinect, identifies objects and measures several of their properties. The collected data for each frame are then made available to network clients through HTTP wrapped in JSON arrays. Its main purpose is to enable web developers to develop computer vision apps using javascript &amp; HTML. The protocol used is standard HTTP though so other languages can take advantage of it as well. It also provides the raw camera frames as JPEG snapshots or MJPEG streams.
<p>
<a> </a>
</p><h2>OPTIONS</h2>

<dl>
<dt><b>-p </b><i>PORT</i></dt><dd>
Sets the port the server should listen to. Default is 6661.<br>
</dd><dt><b>-l </b><i>ADDRESS</i></dt><dd>
Sets the address the server should listen to. Default is INADDR_ANY.<br>
</dd><dt><b>-d </b><i>DEVICE</i></dt><dd>
Sets the kinect device to use. You can either supply an index id(0-9) or the serial of a specific kinect. The special value "list" prints the serials of all kinected kinects. Default is 0, first device by index.<br>
</dd><dt><b>-j </b><i>MODE</i></dt><dd>
Sets the JPEG quality. Range is 25-75. Default is 50.<br>
</dd><dt><b>-m </b><i>MASK</i></dt><dd>
The first 3 bits of the provided integer MASK toggle multipart output for data, grayscale(depth)JPEG and rgb JPEG output respectively. Multipart data responses are handled correctly only on firefox(By setting the property multipart as true on the xmlhttprequest object). In the case of the JPEG outputs it toggles bettween single images and MJPEG streaming. Range is 0-7. Default is 6 which means: poll for data(no multipart) and MJPEG streams for both cameras.<br>
</dd><dt><b>-s </b><i>SECRET</i></dt><dd>
Sets a secret passphrase to be used for securing the live config functionality of the server.  A value of "0" enables the feature but without the need to authenticate. Live config changes are disabled by default.<br>
</dd><dt><b>-f </b><i>FILE</i></dt><dd>
Sets the file to load  the background reference frame from. The format is standard pgm(the extension is NOT added automatically).<br>
</dd><dt><b>-F </b><i>FILE</i></dt><dd>
Sets the file to dump the background reference file to.<br>
</dd><dt><b>-o </b><i>FILE</i></dt><dd>
Reads the contents of the specified file to form a whitelist of origins that can issue requests to the server. Default is to allow all origins.<br>
</dd><dt><b>-i </b><i>FILE</i></dt><dd>
Reads the contents of the specified file to form a whitelist of IPs that can connect to the server from a comma separated list. Default is to allow all IPs.<br>
</dd><dt><b>-n</b></dt><dd>
No rgb video. Disables the rgb camera which gives a small performance boost.<br>
</dd><dt><b>-b</b></dt><dd>
Big messages. By default the max size for the JSON messages of the server is 4KB which can fit ~25 concurrent blobs per message. This option raises the limit to 16KB allowing ~100 blobs.<br>
</dd><dt><b> The following options can also be changed live through http requests</b></dt><dd>
</dd><dt><b>-v</b></dt><dd>
Prints the version of the program.<br>
<p>
</p></dd><dt><b>-e </b><i>INT</i></dt><dd>
Background extraction. This option controls the mode of operation for the background extraction. A value of 0 disables extraction, everything scanned withing the boundaries described by the following options will be taken into account. A positive value forces the server to create a reference frame(controlled by the -r option, see below) and report just the differences on subsequent frames. A negative value the server also utiilizes a background frame but scans with depth boundaries(-z,-Z options) relative to the background frame. Eg along with -z 30 -Z 120 the server will scan for blobs that are from 30 to 120 mm from the wall(or whatever was captured in the background frame). The absolute value of the option adds a safety margin that helps to clear noise that results from the instability of depth reporting from the kinect. Start low and raise until the result satisfies. The default value is 0, no extraction.<br>
</dd><dt><b>-r </b><i>FRAMES</i></dt><dd>
Set the number of frames to take into account while creating the background reference frame. Range is 1-999. Processing multiple frames leads to better quality of the reference frame and the extraction process(See above). While constructing the reference frame the kinect's led will blink orange-red. A value of 1 won't actually process a frame but just reset the depth maps and clear up the depth buffer used for generating the grayscale JPEG(see USAGE). Range is 1-9999. The default is 32, about one second worth of frames.<br>
</dd><dt><b>-x </b><i>PIXEL</i></dt><dd>
Set the lower bound for the X axis scan. Range is 0-632. Will be rounded to a multiple of 8. Default is 0.<br>
</dd><dt><b>-X </b><i>PIXEL</i></dt><dd>
Set the higher bound for the X axis scan. Range is 0-632. Will be rounded to a multiple of 8. Default is 632.<br>
</dd><dt><b>-y </b><i>PIXEL</i></dt><dd>
Set the lower bound for the Y axis scan. Range is 0-478. Default is 0.<br>
</dd><dt><b>-Y </b><i>PIXEL</i></dt><dd>
Set the higher bound for the Y axis scan. Range is 0-479. Default is 479.<br>
</dd><dt><b>-z </b><i>MILLIMETRES</i></dt><dd>
Set the low threshold for the Y axis(depth) scan in mm. Range is 0-9999. Default is 30.<br>
</dd><dt><b>-Z </b><i>MILLIMETRES</i></dt><dd>
Set the high threshold for the Y axis scan in mm. Range is 1-9999. Default is 1340.<br>
</dd><dt><b>-c </b><i>PIXELS</i></dt><dd>
Set the minimum pixel count for an object to be reported. Range is 1-300000. Default is 1024.<br>
</dd><dt><b>-C </b><i>PIXELS</i></dt><dd>
Set the maximum pixel count for an object to be reported. Default is 0, check disabled.<br>
</dd><dt><b>-f </b><i>DUMMY</i></dt><dd>
When used in the context of live config(See below), it forces a reload of the reference frame from/if the file was specified on startup. DUMMY means that an argument has to be passed in the query string but is not taken into account.<br>
</dd><dt><b>-F </b><i>DUMMY</i></dt><dd>
When used in the context of live config(See below), it forces a dump of the reference frame to/if the file was specified on startup with the -F option. DUMMY means that an argument has to be passed in the query string but is not taken into account.<br>
</dd><dt><b>-o </b><i>DUMMY</i></dt><dd>
When used in the context of live config(See below), it forces a reload of the origin list from/if the file was specified on startup with the -o option. DUMMY means that an argument has to be passed in the query string but is not taken into account.<br>
</dd><dt><b>-i </b><i>DUMMY</i></dt><dd>
When used in the context of live config(See below), it forces a reload of the IP list from/if the file was specified on startup with the -i option. DUMMY means that an argument has to be passed in the query string but is not taken into account.<br>
</dd><dt><b>-a </b><i>ANGLE</i></dt><dd>
Moves the motor to the specified angle. After the motor moves to the specified position(Indicated by the last element of the HEADER, see below) you should reconstruct the reference frame(-r option) if using background extraction. Range is -31 to 31.<br>
<p>
</p></dd></dl>
<a> </a>
<h2>USAGE</h2>

<p>
The server speaks the standard http protocol. Clients retrieve the data through xmlhttprequests or the &lt;img&gt; tag. Requests for any path that starts with anything else than /1 and /2(And even that if you have explicitly disabled RGB with the -n option) return the tracking data for the current frame in the form of a JSON array that's composed of an 16 element header followed by several 32 element packs, one for every detected blob. The formats for these are detailed in the next sections. The following paths are special:<br>
</p><p>
</p><dl>
<dt><b>/0?</b><i>QUERY_STRING</i></dt><dd>
This path allows live configuration changes to be performed to the server. All alowable command line options can be used here in the form of a query string eg. /0?z=1000&amp;Z=2000 would force the engine to threshold from 1000 to 2000 millimetres. By default this functionality is disabled. You can enable it without authentication by specifying the option -s 0 on the command line. If -s is set to a string authentication is enabled which works as follows. The second element from the header in the JSON data output of the server(see HEADER FORMAT below) is a timestamp that must be concatenated with  the string provided to -s (&lt;SECRET&gt;&lt;TIMESTAMP&gt;) and the result md5 hashed. The hash must then be passed along with the rest of the query arguments as an s=&lt;HASH&gt; for the commands to take effect. After every succesful request the hash timestamp will change requiring a repeat of the process for subsequent requests. The design does not take into account concurrent requests from multiple clients.<br>
</dd><dt><b>/1</b></dt><dd>
This path provides 640x480 grayscale JPEGs created from the, thresholded, depth input. If multipart is enabled through the -m option(it is by default) you'll get an MJPEG stream else single snapshots per request. {WARNING} The grayscale stream is built as part of the blob tracking process. The implication of this is that you must concurently poll for the JSON data or else the JPEG stream will stall. The implication of this is that you should set the image src after you start polling for the JSON data.<br>
</dd><dt><b>/2</b></dt><dd>
This path provides 640x480 RGB JPEGs created from the video camera input. If multipart is enabled through the -m option(it is by default) you'll get an MJPEG stream else single snapshots per request. You'll have to manually shift parts of the depth JPEGs to match the rgb camera using the value provided as the last element of the data pack for each blob(See BLOB FORMAT below). The x and y shifting modifiers can be derived from this value like this: y=&lt;VAL&gt;/640 and x=&lt;VAL&gt;%640.  If the rgb functionality was disabled with the use of the -n command line option, the regular JSON response will be returned instead.<br>
</dd><dt><b>NOTICE</b></dt><dd>
If several requests to the server have exactly the same paths, the browser will serialize the requests lowering the effective rate for each request. To avoid that you should request a unique path for every simultaneous request. Only the first character matters for selecting the type of data returned. So, a path of "/test3243434" will return the tracking data, a "/1dfdfdfdf" will return grayscale JPEGs and a "0ertdgf?z=..." will perform live changes.<br>
<p>
</p><p>
</p><p>
</p></dd></dl>
<a> </a>
<h2>HEADER FORMAT</h2>

<br>
The first 16 elements of the JSON array from the server's response will always be present, even if no blobs were detected. Each of its elements are described below by index:<br>
<p>
</p><dl>
<dt><b>0</b></dt><dd>
Timestamp of the current frame as provided by the kinect.<br>
</dd><dt><b>1</b></dt><dd>
Timestamp of the last configuration change. See path /0 above.<br>
</dd><dt><b>2</b></dt><dd>
Background extraction mode. See -e option.<br>
</dd><dt><b>3</b></dt><dd>
Low x axis bound for the raster scan. See -x option.<br>
</dd><dt><b>4</b></dt><dd>
High x axis bound for the raster scan. See -X option.<br>
</dd><dt><b>5</b></dt><dd>
Low y axis bound for the raster scan. See -y option.<br>
</dd><dt><b>6</b></dt><dd>
High y axis bound for the raster scan. See -Y option.<br>
</dd><dt><b>7</b></dt><dd>
Low depth threshold. See -z option.<br>
</dd><dt><b>8</b></dt><dd>
High depth threshold. See -Z option.<br>
</dd><dt><b>9</b></dt><dd>
Low pixel count limit for blob filtering. See -c option.<br>
</dd><dt><b>10</b></dt><dd>
High pixel count limit for blob filtering. See -C option.<br>
</dd><dt><b>11</b></dt><dd>
Accelerometer X value in G.<br>
</dd><dt><b>12</b></dt><dd>
Accelerometer Y value in G.<br>
</dd><dt><b>13</b></dt><dd>
Accelerometer Z value in G.<br>
</dd><dt><b>14</b></dt><dd>
Motor angle as reported by the kinect.<br>
</dd><dt><b>15</b></dt><dd>
Motor state. 0-Not moving, 4-Moving.<br>
<p>
</p></dd></dl>
<a> </a>
<h2>BLOB FORMAT</h2>

<p>
After the 16 elements of the header comes the information for the detected blobs in fixed length(32) element packs. The structure of these packs is described below by index:<br>
</p><p>
</p><dl>
<dt><b>0</b></dt><dd>
x coordinate of the geometric center of the object.<br>
</dd><dt><b>1</b></dt><dd>
y coordinate of the geometric center of the object.<br>
</dd><dt><b>2</b></dt><dd>
Average depth of all object's pixels.<br>
</dd><dt><b>3</b></dt><dd>
Background depth at the geometric center of the object.<br>
</dd><dt><b>4</b></dt><dd>
x coordinate of the leftmost point of the object.<br>
</dd><dt><b>5</b></dt><dd>
y coordinate of the leftmost point of the object.<br>
</dd><dt><b>6</b></dt><dd>
Depth of the leftmost point of the object.<br>
</dd><dt><b>7</b></dt><dd>
Background depth at the leftmost point of the object.<br>
</dd><dt><b>8</b></dt><dd>
x coordinate of the rightmost point of the object.<br>
</dd><dt><b>9</b></dt><dd>
y coordinate of the rightmost point of the object.<br>
</dd><dt><b>10</b></dt><dd>
Depth of the rightmost point of the object.<br>
</dd><dt><b>11</b></dt><dd>
Background depth at the rightmost point of the object.<br>
</dd><dt><b>12</b></dt><dd>
x coordinate of the topmost point of the object.<br>
</dd><dt><b>13</b></dt><dd>
y coordinate of the topmost point of the object.<br>
</dd><dt><b>14</b></dt><dd>
Depth of the topmost point of the object.<br>
</dd><dt><b>15</b></dt><dd>
Background depth at the topmost point of the object.<br>
</dd><dt><b>16</b></dt><dd>
x coordinate of the bottommost point of the object.<br>
</dd><dt><b>17</b></dt><dd>
y coordinate of the bottommost point of the object.<br>
</dd><dt><b>18</b></dt><dd>
Depth of the bottommost point of the object.<br>
</dd><dt><b>19</b></dt><dd>
Background depth at the bottommost point of the object.<br>
</dd><dt><b>20</b></dt><dd>
x coordinate of the point of the object nearest to the camera.<br>
</dd><dt><b>21</b></dt><dd>
y coordinate of the point of the object nearest to the camera.<br>
</dd><dt><b>22</b></dt><dd>
Depth of the point of the object nearest to the camera.<br>
</dd><dt><b>23</b></dt><dd>
Background depth at the point of the object nearest to the camera.<br>
</dd><dt><b>24</b></dt><dd>
x coordinate of the point of the object furthest from the camera.<br>
</dd><dt><b>25</b></dt><dd>
y coordinate of the point of the object furthest from the camera.<br>
</dd><dt><b>26</b></dt><dd>
Depth of the point of the object furthest from the camera.<br>
</dd><dt><b>27</b></dt><dd>
Background depth at the point of the object furthest from the camera.<br>
</dd><dt><b>28</b></dt><dd>
Pixel count of the object.<br>
</dd><dt><b>29</b></dt><dd>
The count of continuous horizontal lines(runs) that compose the object.<br>
</dd><dt><b>30</b></dt><dd>
Y coordinate that indicates the area of greatest concentration of runs.<br>
</dd><dt><b>31</b></dt><dd>
Shifting modifier for the rgb mask (See the description for path /2 on USAGE).<br>
<p>
</p><p>
</p></dd></dl>
<a> </a>
<h2>EXAMPLES</h2>

<p>
Check <a href='http://www.intrael.com'><a href='http://www.intrael.com'>http://www.intrael.com</a></a>
</p><p>
<a> </a>
</p><h2>SEE ALSO</h2>

<p>
Check <a href='http://www.openkinect.org'><a href='http://www.openkinect.org'>http://www.openkinect.org</a></a>
</p><p>
</p><p>
<a> </a>
</p><h2>AUTHOR</h2>

<p>
Yannis Gravezas (<a href='mailto:wizgrav@gmail.com'>wizgrav@gmail.com</a>)<br>
</p><p>
<a> </a>
</p><h2>LICENSE</h2>

<p>
This program is free software: you can redistribute it and/or modify<br>
it under the terms of the GNU General Public License as published by<br>
the Free Software Foundation, either version 3 of the License, or<br>
(at your option) any later version.<br>
</p><p>
This program is distributed in the hope that it will be useful,<br>
but WITHOUT ANY WARRANTY; without even the implied warranty of<br>
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the<br>
GNU General Public License v3 for more details.<br>
</p><p>
You should have received a copy of the GNU General Public License v3<br>
along with this program.  If not, see <a href='http://www.gnu.org/licenses/gpl-3.0.txt'><a href='http://www.gnu.org/licenses/gpl-3.0.txt'>http://www.gnu.org/licenses/gpl-3.0.txt</a></a>;.<br>
</p><p>
</p><p>

</p>

<hr>

