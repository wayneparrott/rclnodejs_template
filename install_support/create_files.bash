
#setup envvars from cmake
CMAKE_CURRENT_SOURCE_DIR=$1
PROJECT_NAME=$2
EXECUTABLE_NAME=$3

INSTALL_SUPPORT_DIR=${CMAKE_CURRENT_SOURCE_DIR}/install_support
INSTALL_TMP_DIR=${INSTALL_SUPPORT_DIR}/tmp


#create tmp dir for executable file and launch dir structure
mkdir -p ${INSTALL_TMP_DIR}/launch

#create executable file
sed -e s/_PKG_/${CMAKE_CURRENT_SOURCE_DIR////\\/}/g ${INSTALL_SUPPORT_DIR}/runexecutable.template.sh > \
	${INSTALL_TMP_DIR}/${EXECUTABLE_NAME}

#create launch dir and file
sed -e s/_PKG_/${PROJECT_NAME}/  -e s/_EXECUTABLE_/${EXECUTABLE_NAME}/ \
    ${INSTALL_SUPPORT_DIR}/launchtemplate.launch.py > ${INSTALL_TMP_DIR}/launch/${EXECUTABLE_NAME}.launch.py

