# configure the following files
#   package.json
#   package.xml
#   CMakeLists.txt
#   CMakeLists_rclnode_project.txt
#

import os
import sys
import time

# requried commandline args from CMake custom install task
EXECUTABLE_NAME = sys.argv[1]
PACKAGE_NAME = os.path.basename( os.getcwd() )
TEMPLATE_DIR = os.path.join(os.getcwd(), 'install_support', 'templates')

print("Updating project files")
print("  PACKAGE_NAME", PACKAGE_NAME)
print("  EXECUTABLE_NAME", EXECUTABLE_NAME)

# edit package.json
fin  = open( os.path.join(TEMPLATE_DIR,'package.json'), "r")
fout = open( 'package.json', "w")
for line in fin:
	fout.write(line.replace('_PKG_', PACKAGE_NAME))
fin.close()
fout.close()
print( "  updated package.json")

# edit package.xml
fin  = open( os.path.join(TEMPLATE_DIR,'package.xml'), "r")
fout = open( 'package.xml', "w")
for line in fin:
	fout.write(line.replace('_PKG_', PACKAGE_NAME))
fin.close()
fout.close()
print( "  updated package.xml")

# edit CMakeLists.txt
fin  = open( os.path.join(TEMPLATE_DIR,'CMakeLists.txt'), "r")
fout = open( 'CMakeLists.txt', "w")
for line in fin:
	fout.write(line.replace('_PKG_', PACKAGE_NAME))
fin.close()
fout.close()
print( "  updated CMakeLists.txt")

# edit CMakeLists_rclnodejs.txt
fin  = open( os.path.join(TEMPLATE_DIR,'CMakeLists_rclnodejs_project.txt'), "r")
fout = open( 'CMakeLists_rclnodejs_project.txt', "w")
for line in fin:
	fout.write(line.replace('_EXECUTABLE_', EXECUTABLE_NAME))
fin.close()
fout.close()
print( "  updated CMakeLists_rclnodejs_project")

print( "Complete")
