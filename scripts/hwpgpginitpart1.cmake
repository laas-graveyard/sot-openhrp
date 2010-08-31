loadPlugin sotPatternGenerator${DYN_LIB_EXT}
new PatternGenerator pg

# Initialize files and information of the pattern generator.
pg.setVrmlDir ${HRP2_MODEL_DIRECTORY}/
pg.setVrml HRP2JRLmainsmall.wrl 
pg.setXmlSpec ${HRP2_CONFIG_DIRECTORY}/HRP2SpecificitiesSmall.xml
pg.setXmlRank ${HRP2_CONFIG_DIRECTORY}/HRP2LinkJointRankSmall.xml 




