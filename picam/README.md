This library was written by Chris Cummings. The source code is linked to in
Chris' blog post about it:
http://robotblogging.blogspot.co.uk/2013/10/an-efficient-and-simple-c-api-for.html

This was intended for reducing some of the processing overhead and latency
introduced by capturing frames in OpenCV, where to get a YUV888 image we do
this:

    Camera -> MMAL library -> Video4Linux2 driver -> YUV420I to BGR888
        -> OpenCV VideoCapture -> BGR888 to YUV888 -> Process image
        
When really we want this:

    Camera -> MMAL library -> PiCamera instance -> YUV420I to YUV888
        -> Process image

This isn't yet implemented because I was running out of time in my dissertation.
I also need to ask Chris if it's okay to add the source to my git repos and find
out what the license is etc. So for now, you can download and extract the zip
file linked to in his blog post into this directory:
http://www.cheerfulprogrammer.com/downloads/picamtutorial/picamdemo.zip
(The pi_build.sh script does this automatically)
