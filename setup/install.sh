#!/bin/bash
# Install everything we need for linux UBW32 development -
#       - C32 windows binaries & Wine
#       - Programmer & required libs

C32_DEST="/usr/local/lib"
C32_TARBALL="c32.tar.gz"

APT_PKGS="wine libusb-dev build-essential subversion automake libtool xsltproc"

UBW32_DIR="ubw32"
UBW32_BIN="/usr/local/bin/ubw32"

LIBHID="/usr/lib/libhid.so.0"

# Install the Microchip c32 compiler
function install_c32 {
    echo "Installing c32..."

    # Check install dir before overwriting
    if [ -e ${C32_DEST}/c32 ] ; then
        echo "${C32_DEST}/c32 exists, skipping c32 install."
        return 0
    fi

    # Fetch C32, if necessary
    if ! [ -f ${C32_TARBALL} ]; then
        echo "Fetching c32..."
        if ! wget http://ipeet.org/uwrt/${C32_TARBALL} ; then
            echo "Failed to fetch ${C32_TARBALL}!"
            return 1
        fi
    fi

    # Check that C32 has been fetched
    if ! [ -f ${C32_TARBALL} ]; then
        echo "${C32_TARBALL} does not exist!"
        return 1
    fi

    echo "Extracting files..."
    tar -xvzf ${C32_TARBALL} -C ${C32_DEST}

    echo "Done extracting.  Setting permissions..."
    chmod 0755 ${C32_DEST}/c32/*

    echo "Finished installing C32."
    return 0
}

# Grab some packages with apt-get
function fetch_packages {
    echo "Getting required packages..."
    if which apt-get > /dev/null ; then
        apt-get install -y ${APT_PKGS}
        return $?
    else
        echo "This system doesn't have apt-get.  You will need to get the following yourself:"
        echo ${APT_PKGS}
        return 1
    fi
}

# Libhid isn't in apt anymore, as of ubuntu 10.10.  Charming. 
# Fetch, compile, and install from source
function install_libhid {
    echo "Installing libhid from source... (Thanks a bunch, debian!)"
    if [ -e ${LIBHID} ] ; then
        echo "It appears you already have libhid.  Skipping install."
        return 0
    fi

    # Check it out
    if ! svn co svn://svn.debian.org/libhid/trunk libhid-svn ; then
        echo "Failed to fetch libhid source."
        return 1
    fi

    # Build and install
    cd libhid-svn
    ./autogen.sh --disable-werror
    if ! make install ; then
        echo "Failed to build libhid."
        cd ..
        return 1
    fi
    # Aaand libhid doesn't get installed where dynamic linker looks, so we need a softlink
    ln -s /usr/local/lib/libhid.so.0.0.0 /usr/lib/libhid.so.0

    cd ..

    echo "libhid installed."
    return 0
}

# Compile and install programmer
function install_ubw32 {
    echo "Building programmer tool..."
    if ! [ -d ${UBW32_DIR} ] ; then
        echo "Directory ${UBW32_DIR} does not exist, skipping ubw32 build and install."
        return 1
    fi
    if [ -e ${UBW32_BIN} ] ; then
        echo "It appears you already have ubw32 installed.  Skipping install."
        return 0
    fi

    if make -C ${UBW32_DIR} clean && make -C ${UBW32_DIR} && make -C ${UBW32_DIR} install ; then
        echo "Programmer successfully built and installed."
        return 0
    else
        echo "Failed to build programmer"
        return 1
    fi
}

# Check that we're running as root 
if [ $(whoami) != "root" ] ; then
    echo "Need root!"
    exit 1
fi

if install_c32 && fetch_packages && install_libhid && install_ubw32; then
    echo -e "\nInstall successful."
    exit 1
else 
    echo -e "\nInstall failed."
    exit 0
fi

