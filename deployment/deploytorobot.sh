# This script is used internally in the JRL to copy the binary files to the robot's computer.

USEROPENROBOTSPATH=${ROBOTPKG_BASE}
WORKINGDIRECTORY=/tmp/
TARGETPATH=grxuser@hrp2010c:/home/grxuser/devel/openrobots
SCPCMD="rsync -l"
SCPCMD_NOLINKS=scp
CPCMD=/bin/cp
USERNAME=`whoami`

echo $USERNAME

if ![ -n "$OPENHRPHOME" ]; then
  OPENHRPHOME=/home/${USERNAME}/src/OpenHRP-3.0.5
fi;

${CPCMD} -r ${USEROPENROBOTSPATH}/script ${WORKINGDIRECTORY}
WDSCRIPT=${WORKINGDIRECTORY}/script
chmod u+w ${WDSCRIPT}/* -R

TARGETLIB=/home/grxuser/devel/openrobots/lib/
ORIGINSHARE=/home/stasse/devel/openrobots/share/
echo

function ChangeFiles()
{
    cd $1
    for i in `ls`; do
	if [ -f $i ]; then
	    filetooperate=$i
	    echo ${filetooperate};

	    cp ${filetooperate} tmp_$i;
	    chmod u+rw tmp_$i;

	    cat tmp_$i | sed "s@/home/${USERNAME}/src/OpenHRP@/home/grxuser/src/OpenHRP@g" > tmp3_$i;
	    chmod u+rw tmp3_$i;

	    cat tmp3_$i | sed "s@/home/${USERNAME}/src/OpenHRP@/home/grxuser/src/OpenHRP@g" > tmp2_$i;
	    chmod u+rw tmp2_$i;

	    cat tmp2_$i | sed "s@${USEROPENROBOTSPATH}@/home/grxuser/devel/openrobots/@g" > $i;
	    rm tmp*_$i -f
	elif [ -d $i ]; then
	    echo
	    echo going into directory $i;
	    ChangeFiles $i
	fi;
    done;
    cd -
}

ChangeFiles ${WORKINGDIRECTORY}/script

# python scripts

HRPSCRIPTDIR=${OPENHRPHOME}/Controller/IOserver/robot/HRP2JRL/script/
TARGETPYTHON=grxuser@192.168.10.201:/home/grxuser/src/OpenHRP/Controller/IOserver/robot/HRP2JRL/script/
WDPYTHON=/tmp/python
mkdir -p ${WDPYTHON}
cp ${HRPSCRIPTDIR}/sot*.py ${WDPYTHON}


cd ${WDPYTHON}

for i in `ls`; do
    if [ -f $i ]; then
  chmod 644 $i;
	cp $i tmp_$i;
	cat tmp_$i | sed "s/${USERNAME}/grxuser/g" > $i;
	rm -f tmp*_$i;
    fi;
done;

cd -


#scp python

echo ${WDPYTHON}
echo ${TARGETPYTHON}
${SCPCMD} ${WDPYTHON}/* ${TARGETPYTHON}


rsh grxuser@hrp2010c mkdir -p ${TARGETLIB}/plugin
rsh grxuser@hrp2010c mkdir -p ${TARGETSHARE}

echo "Copying scripts and openrobots files"
#scp openrobots

${SCPCMD} -r ${WDSCRIPT} ${TARGETPATH}

${SCPCMD} -r ${USEROPENROBOTSPATH}/lib/plugin ${TARGETPATH}/lib

${SCPCMD} -r ${USEROPENROBOTSPATH}/bin ${TARGETPATH}

function CopyAndLinkFiles()
{

    TARGETBIN=/home/grxuser/src/OpenHRP/Controller/IOserver/robot/HRP2JRL/bin

    if  [ "$1" = "dyn" ] ; then 
      v=`ls ${USEROPENROBOTSPATH}/lib/libdynamics*`;
    elif [ "$1" = "hrp2dyn" ]; then
      v=`ls ${USEROPENROBOTSPATH}/lib/libhrp2Dynamics*`;	
    elif [ "$1" = "sot" ]; then
      v=`ls ${USEROPENROBOTSPATH}/lib/libsot-core* ${USEROPENROBOTSPATH}/lib/libdynamic-graph*`;	
    elif [ "$1" = "walkGenJrl" ]; then
      v=`ls ${USEROPENROBOTSPATH}/lib/libwalkGenJrl*`;	
    elif [ "$1" = "scd" ]; then
      v=`ls ${USEROPENROBOTSPATH}/lib/libscd*`;
    elif [ "$1" = "hrp2opt" ]; then
      v=`ls ${USEROPENROBOTSPATH}/lib/libhrp2-10-optimi*`;
    elif [ "$1" = "gfortran" ]; then 
      v=`ls ${USEROPENROBOTSPATH}/lib/libgfortran*`;
    elif [ "$1" = "boost" ]; then
      v=`ls ${USEROPENROBOTSPATH}/lib/libboost_thread* ${USEROPENROBOTSPATH}/lib/libboost_program_options*`;	
    fi;

    ${SCPCMD} $v ${TARGETPATH}/lib
    for i in $v; do
	if [ -f $i ]; then
	    filetooperate=`basename $i`
	    echo rsh grxuser@hrp2010c ln -sf ${TARGETLIB}/${filetooperate} ${TARGETBIN}/${filetooperate};
	    rsh grxuser@hrp2010c ln -sf ${TARGETLIB}/${filetooperate} ${TARGETBIN}/${filetooperate};
	fi;
    done;
    cd -
}

#${SCPCMD}  ${USEROPENROBOTSPATH}/lib/libdynamics* ${TARGETPATH}/lib
CopyAndLinkFiles "dyn"
#${SCPCMD}  ${USEROPENROBOTSPATH}/lib/libhrp2Dynamics* ${TARGETPATH}/lib
CopyAndLinkFiles "hrp2dyn"
#${SCPCMD}  ${USEROPENROBOTSPATH}/lib/libsot* ${TARGETPATH}/lib
CopyAndLinkFiles "sot"
#${SCPCMD}  ${USEROPENROBOTSPATH}/lib/libwalkGenJrl* ${TARGETPATH}/lib
CopyAndLinkFiles "walkGenJrl"
#CopyAndLinkFiles "scd"
CopyAndLinkFiles "hrp2opt"
#scp plugin
CopyAndLinkFiles "gfortran"
CopyAndLinkFiles "boost"

echo "rsh grxuser@hrp2010c rm /home/grxuser/src/OpenHRP-3.0.5/Controller/IOserver/robot/HRP2JRL/bin/StackOfTasks.so"
rsh grxuser@hrp2010c rm /home/grxuser/src/OpenHRP-3.0.5/Controller/IOserver/robot/HRP2JRL/bin/StackOfTasks.so
${SCPCMD_NOLINKS} $OPENHRPHOME/Controller/IOserver/robot/HRP2JRL/bin/StackOfTasks.so grxuser@hrp2010c:/home/grxuser/src/OpenHRP/Controller/IOserver/robot/HRP2JRL/bin

# Robots description
${SCPCMD_NOLINKS} -r ${ORIGINSHARE}/hrp2_10-small-old grxuser@hrp2010c:/home/grxuser/devel/openrobots/share
${SCPCMD_NOLINKS} -r ${ORIGINSHARE}/hrp2_10-small grxuser@hrp2010c:/home/grxuser/devel/openrobots/share
${SCPCMD_NOLINKS} -r ${ORIGINSHARE}/hrp2_10 grxuser@hrp2010c:/home/grxuser/devel/openrobots/share
