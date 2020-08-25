This is a simple utility which we use for our visualisations
and for generating animations. We generally visualize outputs
of our fast bundler in SLAM++ (http://sf.net/p/slam-plus-plus/).

It is currently in beta - it is fully functional and contains
no known bugs, but the commandline interface will change and
new features will be added.

Data Format
===========

It can display simple text files with one vertex per line. 3D
vertices are interpreted as points, 6D vectices are interpreted
as cameras. The cameras are represented by 3D position, followed
by 3D axis-angle (axis of rotation with magnitude of rotation
angle in radians). The cameras are inverse (they represent
transformation from world space to camera space).

In addition to that, if a .graph file is available, it can
display edges between the vertices. Standard edges, such as
EDGE_PROJECT_P2MC or EDGE_PROJECT_P2SC are expected. The parser
is, however, minimal and easily extensible. The rationale here
is that the viewer is not attempting any parsing of the graph
file beyond the indices, nor does it attempt initializing vertex
positions from it. That is up to SLAM++.

Finally, if marginal covariances are available, they can be
displayed normalized in false colour. Again, one line per vertex
is expected to contain the diagonal of vertex cross-covariance
block.

Features
========

This uses OpenGL ES, and as such could be easily ran on Android
or a similar platform. GraphView has intricate keyframe camera
animation system, which can be used to create flyby animations.
The rendered output has both spatial and temporal antialiassing.

Omnidirectional videos for YouTube
==================================

One of the features is rendering of flyby omnidirectional videos
(also known as "360 degree" videos). To do that, you need to
specify "--omni-video" or "-ov" on the commandline and set aspect
ratio to 2:1, e.g. using "-vw 7168 -vh 3584" for 8k video or
"-vw 4320 -vh 2160" for 4k video. Note that as of Q1 2017,
YouTube does not seem to support 8k at 48 frames per second
(using 25 FPS seems to work fine).

After rendering the frames, specific steps are needed. First,
you need to encode your video to x264, with mp4 container. It
is recommended to use 35-45 Mbps for 4k video at 25 FPS and
53-68 Mbps for 4k video at 48 FPS or higher (use about 85 Mbps
for 8k video).

To encode to mp4 container, you can use mencoder switch
"-of lavf". If you're using HandBrake to recode your video
to mp4, make sure to disable cropping (by default it crops
16 px on top and a bit less on the bottom), otherwise the
area near the top and bottom poles of the view sphere will
be severely distorted.

Finally, you need to use "360 Video Metadata Tool" to insert
the following metadata into the "moov.trak.uuid" box indexed
by uuid "ffcc8263-f855-4a93-8814-587a02521fdd":

    <?xml version="1.0"?>
    <rdf:SphericalVideo xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
                        xmlns:GSpherical="http://ns.google.com/videos/1.0/spherical/">
        <GSpherical:Spherical>true</GSpherical:Spherical>
        <GSpherical:Stitched>true</GSpherical:Stitched>
        <GSpherical:StitchingSoftware>GraphViewer</GSpherical:StitchingSoftware>
        <GSpherical:ProjectionType>equirectangular</GSpherical:ProjectionType>
    </rdf:SphericalVideo>

This format also allows you to set the initial view angles
and cropping.

Only then will YouTube recognize the video as spherical.
Note that there is a draft for "Spherical Video V2 RFC"
that allows specification of the spherical video in cube-
map format rather than lon-lat format. This should allow
considerably faster rendering (the resampling to unwrapped
sphere takes a lot of effort to do without aliasing).

Building
========

In windows, use the Visual Studio workspace. You can also download
pre-built libpng and libjpeg from the Files section on sourceforge.
Note that when building in the 32-bit mode, you will need to disable
safe exception handling (SEH) because GLUT is built without it. This
presents a minor security issue. Always use the 64-bit mode unless
you are specifically targetting an old machine. The 32-bit mode also
severely limits the maximum size of the screenshots.

In Linux, use make. Note that the order of linked libraries only
works with some Linuxes and you may need to edit the makefile in
case you are getting linking errors. This would need a CMakeLists.txt
to be resolved properly.

On OS X, you need to use the Makefile.mac

mv Makefile Makefile.linux
mv Makefile.mac Makefile
make

also, in case you get fatal error: 'png.h' file not found, you
need to:

cd ..
mkdir zlib
cd zlib
curl -L -O http://zlib.net/fossils/zlib-1.2.8.tar.gz
tar -xvzf zlib-1.2.8.tar.gz
cd zlib-1.2.8
./configure
make
sudo make install
cd ../..

mkdir libpng
curl -L -O http://downloads.sourceforge.net/project/libpng/libpng12/older-releases/1.2.8/libpng-1.2.8.tar.bz2
tar -xjf libpng-1.2.8.tar.bz2
cd libpng-1.2.8
cp scripts/makefile.std Makefile
make
sudo make install
cd ../../GraphViewer/

You can also try the current version, it should work just as well.
Version 1.2.8 is fairly old at this point.

If this does not help and you'øe still getting linking errors, try
changing the order of libraries in makefile (see the second paragraph
of Building, or the comments in makefile).
