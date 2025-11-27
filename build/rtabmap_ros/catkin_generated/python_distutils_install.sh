#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/isro/isro_ws/src/rtabmap_ros"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/isro/isro_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/isro/isro_ws/install/lib/python2.7/dist-packages:/home/isro/isro_ws/build/rtabmap_ros/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/isro/isro_ws/build/rtabmap_ros" \
    "/usr/bin/python2" \
    "/home/isro/isro_ws/src/rtabmap_ros/setup.py" \
    egg_info --egg-base /home/isro/isro_ws/build/rtabmap_ros \
    build --build-base "/home/isro/isro_ws/build/rtabmap_ros" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/isro/isro_ws/install" --install-scripts="/home/isro/isro_ws/install/bin"
