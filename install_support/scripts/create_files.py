import os
import sys
import time

# requried commandline args from CMake custom install task
CMAKE_CURRENT_SOURCE_DIR = sys.argv[1]
PROJECT_NAME = sys.argv[2]
EXECUTABLE_NAME = sys.argv[3]

# local paths
INSTALL_SUPPORT_DIR = os.path.join(CMAKE_CURRENT_SOURCE_DIR, 'install_support')
INSTALL_TMP_DIR = os.path.join(INSTALL_SUPPORT_DIR, 'tmp')
LAUNCH_DIR = os.path.join(INSTALL_TMP_DIR, 'launch');

# create tmp dir for executable file and launch dir structure
os.makedirs(LAUNCH_DIR, exist_ok=True)
time.sleep(1) #hack to wait for dirs to be created before writing files into them

# create executable file
# equivalent of sed script
#    sed -e s/_PKG_/${CMAKE_CURRENT_SOURCE_DIR////\\/}/g ${INSTALL_SUPPORT_DIR}/runexecutable.template > \
#	     ${INSTALL_TMP_DIR}/${EXECUTABLE_NAME}
fin  = open( os.path.join(INSTALL_SUPPORT_DIR, 'runexecutable.template'), "r")
fout = open( os.path.join(INSTALL_TMP_DIR, EXECUTABLE_NAME), "w+")
for line in fin:
	fout.write(line.replace('_PKG_INSTALL_PATH_', CMAKE_CURRENT_SOURCE_DIR))

fin.close()
fout.close()


# create launch file
# equivalent of sed script
#    sed -e s/_PKG_/${PROJECT_NAME}/  -e s/_EXECUTABLE_/${EXECUTABLE_NAME}/ \
#        ${INSTALL_SUPPORT_DIR}/launchtemplate.launch.py > ${INSTALL_TMP_DIR}/launch/${EXECUTABLE_NAME}.launch.py
fin = open( os.path.join(INSTALL_SUPPORT_DIR, 'launchtemplate.launch.py'), "r")
fout = open( os.path.join(LAUNCH_DIR, EXECUTABLE_NAME+'.launch.py'), "w+")

for line in fin:
	fout.write( 
		line.replace('_PKG_', PROJECT_NAME).
		replace('_EXECUTABLE_', EXECUTABLE_NAME).
		replace('_RVIZ_CONFIG_PATH_', os.path.join(CMAKE_CURRENT_SOURCE_DIR, 'config', 'rvizconfig.rviz') )
	)

fin.close()
fout.close()

