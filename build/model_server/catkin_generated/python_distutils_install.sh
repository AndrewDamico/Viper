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

echo_and_run cd "/home/andrew/viper/src/model_server"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/andrew/viper/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/andrew/viper/install/lib/python3/dist-packages:/home/andrew/viper/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/andrew/viper/build" \
    "/usr/bin/python3" \
    "/home/andrew/viper/src/model_server/setup.py" \
     \
    build --build-base "/home/andrew/viper/build/model_server" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/andrew/viper/install" --install-scripts="/home/andrew/viper/install/bin"
