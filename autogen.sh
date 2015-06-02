#!/bin/sh

./autoclean.sh

rm -f configure

rm -f Makefile.in

rm -f config.guess
rm -f config.sub
rm -f install-sh
rm -f missing
rm -f depcomp

#sudo yum install -y hunspell-devel

if [ 0 = 1 ]; then
autoscan
else
#cd pflib && ./autogen.sh && cd ..

touch NEWS
touch README
touch AUTHORS
touch config.h.in
touch ChangeLog

libtoolize --copy --force
aclocal
automake -a -c
autoconf

#./configure --enable-debug
./configure --enable-debug=no --prefix=`pwd`/aaa
#./configure --disable-debug
make clean
make ChangeLog
make dist
#make

fi
