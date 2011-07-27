#get the mode flag from the command line
#default to 'release' if the user didn't specify
mymode = ARGUMENTS.get('mode', 'debug')   #holds current mode

#check if the user has been naughty: only 'debug' or 'release' allowed
if not (mymode in ['debug', 'release']):
    print "Error: expected 'debug' or 'release', found: " + mymode
    Exit(1)

#tell the user what we're doing
print '**** Compiling in ' + mymode + ' mode...'

debugflags = ['-g', '-DDEBUG']   #extra compile
releaseflags = ['-O2']          #extra compile

env = Environment(CC = 'i586-pc-linux-gnu-gcc', CXX = 'i586-pc-linux-gnu-g++', LIBS=['expat'])

if mymode == 'debug':
    env.Append(CCFLAGS=debugflags)
else:
    env.Append(CCFLAGS=releaseflags)

#make sure the sconscripts can get to the variables
Export('env', 'mymode')

#put all .sconsign files in one place
env.SConsignFile()
env.Program('testpov', source=['testpov.cpp'])
