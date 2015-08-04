< envPaths 

dbLoadDatabase("$(TOP)/dbd/KsApp.dbd")
KsApp_registerRecordDeviceDriver(pdbbase) 

epicsEnvSet("PREFIX", "Qi2:")
epicsEnvSet("PORT",   "KS1")
epicsEnvSet("QSIZE",  "200")
epicsEnvSet("XSIZE",  "4908")
epicsEnvSet("YSIZE",  "3264")
epicsEnvSet("NCHANS", "2048")
epicsEnvSet("CBUFFS", "500")
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")


#KsCamConfig: PORT, CameraIndexNumber, maxBuffers, maxMemory, priority, stackSize
#CameraIndexNumber:
# with camera attached:
# 0 : Attached camera 
# 1 : DS-Ri2(Simulation)
# 2 : DS-Qi2(Simulation)
# without camera attached:
# 0 : DS-Ri2(Simulation)
# 1 : DS-Qi2(Simulation)
KsCamConfig("$(PORT)", 0, 200, 0, 0 , 0 )

dbLoadRecords("$(ADCORE)/db/ADBase.template",   "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(ADCORE)/db/NDFile.template",   "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
# Note that Ks.template must be loaded after NDFile.template to replace the file format correctly
dbLoadRecords("$(AREA_DETECTOR)/ADNikonKs/db/Ks.template","P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Create a standard arrays plugin
NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, 0)
#dbLoadRecords("$(ADCORE)/db/NDPluginBase.template","P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0")
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)TYPE=Int16,FTVL=SHORT,NELEMENTS=16019712")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/iocBoot/commonPlugins.cmd

set_requestfile_path("$(ADNIKONKS)/KsApp/Db")

#asynSetTraceMask("$(PORT)",0,255)
#asynSetTraceMask("$(PORT)",0,3)


iocInit()
# save things every thirty seconds
create_monitor_set("auto_settings.req", 5,"P=$(PREFIX)")

