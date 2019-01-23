# 
# Ubitrack - Library for Ubiquitous Tracking
# Copyright 2019, Technische Universitaet Muenchen, and individual
# contributors as indicated by the @authors tag. See the
# copyright.txt in the distribution for a full listing of individual
# contributors.
# 
# This is free software; you can redistribute it and/or modify it
# under the terms of the GNU Lesser General Public License as
# published by the Free Software Foundation; either version 2.1 of
# the License, or (at your option) any later version.
# 
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public
# License along with this software; if not, write to the Free
# Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
# 02110-1301 USA, or see the FSF site: http://www.fsf.org.
# 

import argparse
import os

# parse command line
parser = argparse.ArgumentParser(description='''
This script takes a directory with EuRoC images and generates an Ubitrack timestamp file.
''')
parser.add_argument('directory', help='directory containing EuRoC images, which are named with timestamps')
parser.add_argument('timestamp_filename', help='name of timestamp file')
args = parser.parse_args()

fileList = os.listdir(args.directory)
fileList.sort()
lineList = []
for fileName in fileList:
   parts = fileName.split('.')
   if parts[1].lower() == "png":
      line = parts[0] + " \"" + fileName + "\"\n" 
      lineList.append(line)

tsFile = open(args.timestamp_filename, 'w')
tsFile.writelines(lineList)
