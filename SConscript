from building import *

cwd          = GetCurrentDir()
include_path = [cwd]
src          = []

if GetDepend(['GLP_USING_ICMB']):
	src	+= ['icmb.c']

if GetDepend(['ICMB_SAMPLE']):
	src	+= ['icmb_sample.c']

group = DefineGroup('icmb', src, depend = ['GLP_USING_ICMB'], CPPPATH = include_path)

Return('group')
