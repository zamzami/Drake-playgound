### mexIPOPT on OSX
**by Enrico Bertolazzi**

**Install IPOPT from source**

To install IPOP from source read `README_how_to_install_ipopt.md`.
This is the preferred way cause it is possibile to produce a mex
file mainly linked with static libraries avoiding conflict with MATLAB dynamic libraries.

**Install IPOPT using homebrew**

To install mexIPOPT on OSX one easy way is to use Homebrew.
Donwload Homebrew from `http://brew.sh` and install.
Then on a shell windows type

~~~
brew install ipopt
~~~

installation may use some options, 
for example to install IPOPT using openblas support:

~~~
brew install ipopt --with-openblas
~~~

for a list of options type

~~~
brew info ipopt
~~~

IPOPT depends on `mumps` which by default use MPI.
It is possible to use `mumps` without MPI support

~~~
brew install mumps --without-mpi
~~~

**MATLAB and Xcode 7.0**

If you have upgraded Xcode to version 7.0 mex file 
do not compile. 

The problem is that Xcode 7 change SDK from MacOSX10.10.sdk 
to MacOSX10.11.sdk.
Matlab search SDK only up to MacOSX10.10.sdk.
A workaround is to modify the files clang_maci64.xml 
and clang++_maci64.xml inside MATLAB application to search also this SDK.
The files are in the directory 

/Applications/MATLAB_R2015a.app/bin/maci64/mexopts/

In this file every time you find a row with

`<dirExists name="$$/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.10.sdk" />`

change with

`<dirExists name="$$/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.10.sdk" />`
`<dirExists name="$$/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.11.sdk" />`

After that you need to restart MATLAB and execute

`mex -setup C`
`mex -setup C++`

in order that MATLAB is aware of the modifications.
