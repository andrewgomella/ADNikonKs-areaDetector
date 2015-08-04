/* Ks.cpp
 *
 * This is a driver for Nikon Qi2 and Nikon Ri2 cameras.
 *
 * Author: Andrew Gomella
 *         NIH
 *
 * Written: April 2015
 *
 * Adapted from Mark River's Prosilica/PointGrey Drivers
 */


//Standard Includes
#include <map>
#include <atlstr.h> //needed for the CString declaration

//Epics Includes
#include "ADDriver.h"
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <epicsExit.h>

//Nikon KsCamSDK Inlcudes
#include "KsCam.h"
#include "KsCamCommand.h"
#include "KsCamEvent.h"
#include "KsCamFeature.h"
#include "KsCamImage.h"

//Nikon SDK defines
#define countof(A) (sizeof(A) / sizeof((A)[0]))
#define DEF_GOSA                    0.0000001
#define DEF_UNIT                    1000000.0
/* I am not sure if this number matters in practice as I have never seen the buffer go 
   over 1. Perhaps it only matters when using the MultiExposure Feature */
#define DEF_DRIVER_BUFFER_NUM       5 
#define DEF_FRAME_SIZE_MAX          (4908 * (3264 + 1) * 3)

/* KsCam driver specific parameters */
#define ExposureModeString           "EXPOSURE_MODE"
#define ExposureBiasString           "EXPOSURE_BIAS"
#define MeteringModeString           "METERING_MODE"
#define MeteringAreaLeftString       "METERING_AREA_LEFT"
#define MeteringAreaTopString        "METERING_AREA_TOP"
#define MeteringAreaWidthString      "METERING_AREA_WIDTH"
#define MeteringAreaHeightString     "METERING_AREA_HEIGHT"
#define ExposureTimeLimitString      "EXPOSURE_TIME_LIMIT"
#define GainLimitString              "GAIN_LIMIT"
#define CaptureModeString            "CAPTURE_MODE"
#define BrightnessString             "BRIGHTNESS"

#define SharpnessString              "SHARPNESS"            //Ri2 only
#define HueString                    "HUE"                  //Ri2 only
#define SaturationString             "SATURATION"           //Ri2 only
#define WhiteBalanceRedString        "WHITE_BALANCE_RED"    //Ri2 only
#define WhiteBalanceBlueString       "WHITE_BALANCE_BLUE"   //Ri2 only
#define ColorPresetString            "COLOR_PRESET"         //Ri2 only

#define NikonTriggerModeString       "NIKON_TRIGGER_MODE"
#define TriggerOptionFrameCtString   "TRIGGER_OPTION_FRAME_CT"
#define TriggerOptionDelayString     "TRIGGER_OPTION_DELAY"
#define SignalExposureEndString      "SIGNAL_EXPOSURE_END"
#define SignalTriggerReadyString     "SIGNAL_TRIGGER_READY"
#define SignalDeviceCaptureString    "SIGNAL_DEVICE_CAPTURE"
#define ExposureOutputString         "EXPOSURE_OUTPUT"
#define FormatString                 "FORMAT"
#define ROIXString                   "ROIX"
#define ROIYString                   "ROIY"
#define SoftTriggerString            "SOFT_TRIGGER"
#define TriggerCancelString          "TRIGGER_CANCEL"
#define FrameDroplessString          "FRAME_DROPLESS"
#define SignalEventString            "SIGNAL_EVENT"
#define NikonFrameRateString         "NIKON_FRAME_RATE"
#define FwVersionString              "FW_VERSION"
#define FpgaVersionString            "FPGA_VERSION"
#define Fx3VersionString             "FX3_VERSION"
#define UsbVersionString             "USB_VERSION"
#define DriverVersionString          "DRIVER_VERSION"
#define SDKVersionString             "SDK_VERSION"

/* KsCam Error code strings from KsCam.h */
std::map<int, std::string> KsErrorMap = {{ 0,   "LX_OK"},
                                         {-1,   "LX_ERR_UNEXPECTED"},
                                         {-2,   "LX_ERR_NOTIMPL"},
                                         {-3,   "LX_ERR_OUTOFMEMORY"},
                                         {-4,   "LX_ERR_INVALIDARG"},
                                         {-5,   "LX_ERR_NOINTERFACE"},
                                         {-6,   "LX_ERR_POINTER"},
                                         {-7,   "LX_ERR_HANDLE"},
                                         {-8,   "LX_ERR_ABORT"},
                                         {-9,   "LX_ERR_FAIL"},
                                         {-10,  "LX_ERR_ACCESSDENIED"}
                                        };

/* Driver for Nikon DS-Qi2 and DS-Ri2 cameras using the KsCam SDK library */
static const char *driverName = "KsCam";
class KsCam : public ADDriver
{
public:
    /* Constructor and Destructor */
    KsCam(const char *portName, int cameraNumber, int maxBuffers, size_t maxMemory,
        int priority, int stackSize);
    ~KsCam();

    /* These methods are overwritten from asynPortDriver */
    virtual asynStatus connect(asynUser* pasynUser);
    virtual asynStatus disconnect(asynUser* pasynUser);

    /* These are the methods that we override from ADDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64( asynUser *pasynUser, epicsFloat64 value);

    /* KsCam Event Handler must be public */
    void DoEvent(const lx_uint32 uiCameraHandle, CAM_Event* pstEvent, void* pTransData);

    /* Disconnects the camera */
    static void shutdown(void *arg);

    /* Task for grabbing images in separate process */
    void imageGrabTask();

protected:
    int ExposureMode;
#define FIRST_KSCAM_PARAM ExposureMode
    int ExposureBias;
    int MeteringMode;
    int MeteringAreaLeft;
    int MeteringAreaTop;
    int MeteringAreaWidth;
    int MeteringAreaHeight;
    int ExposureTimeLimit;
    int GainLimit;
    int CaptureMode;
    int Brightness;
    int NikonTriggerMode;
    int TriggerOptionFrameCt;
    int TriggerOptionDelay;
    int SignalExposureEnd;
    int SignalTriggerReady;
    int SignalDeviceCapture;
    int ExposureOutput;
    int Format;
    int ROIX;
    int ROIY;
    int Sharpness;
    int Hue;
    int Saturation;
    int WhiteBalanceRed;
    int WhiteBalanceBlue;
    int ColorPreset;
    int SoftTrigger;
    int TriggerCancel;
    int FrameDropless;
    int SignalEvent;
    int NikonFrameRate;
    int FwVersion;
    int FpgaVersion;
    int Fx3Version;
    int UsbVersion;
    int DriverVersion;
    int SDKVersion;
#define LAST_KSCAM_PARAM SDKVersion
#define NUM_KSCAM_PARAMS ((int)(&LAST_KSCAM_PARAM - &FIRST_KSCAM_PARAM + 1))

private:
    /* These are the methods that are new to this class */
    asynStatus connectCamera();
    asynStatus disconnectCamera();
    asynStatus startCapture();
    asynStatus stopCapture();
    asynStatus grabImage();

    asynStatus updateImageSettings();
    asynStatus SetFloatValue(lx_uint32 uiFeatureId, epicsFloat64 value);
    asynStatus SetIntValue(lx_uint32 uiFeatureId, epicsInt32 value, int option);
    asynStatus SetFeature(lx_uint32 uiFeatureId);
    void       featureChanged(lx_uint32 uiFeatureId);
    void       GetAllFeatures();
    void       GetAllFeaturesDesc();
    void       Command(const lx_wchar* wszCommand);
    void       GetDropless();
    void       onBusReset(lx_uint32 busResetCode, lx_uint32 imageCleared);

    /* utility functions copied from Nikon example code */
    void       PrintAllFeatures();
    lx_wchar*  ConvFeatureIdToName(const lx_uint32 uiFeatureId);
    CString    GetFeatureDispString(CAM_FeatureValue* pstFeatureValue);
    BOOL       IsInRange(lx_int32 iTarget, CAM_FeatureDescRange& stRange);
    BOOL       IsInRange(lx_int32 target, lx_int32 min, lx_int32 max,
                              lx_int32 pitch);
    BOOL       IsInRangeU(lx_uint32 target, lx_uint32 min, lx_uint32 max,
                               lx_uint32 pitch);
    BOOL       IsInRangeU(lx_uint32 uiTarget, CAM_FeatureDescRange& stRange);
    lx_uint32  AdjustExposureTime(lx_uint32 uiValue);
    BOOL       CheckExposureTime(lx_uint32 uiValue);
    void       FrameRateStart();
    void       FrameRate();
    DWORD      GetTickCountQuality();

    //  Device Info ----------------------------------------
    int             camerNumber;
    BOOL            m_isOpened;
    lx_uint32       m_uiDeviceIndex;
    CAM_Device      m_stDevice;
    CAM_Image       stImage;
    lx_uint32       m_uiDeviceCount;

    //  Camera ---------------------------------------------
    lx_uint32               m_uiCameraHandle;
    CAM_CMD_GetFrameSize    m_stFrameSize;

    //  Callback -------------------------------------------
    void*   m_ptrEventTransData;

    //  FrameRate ---------------------------------------------
    DWORD   m_dwStartTick;
    DWORD   m_dwStartTickAve;
    DWORD   m_dwCount;
    DWORD   m_dwCountAve;

    // Feature ---------------------------------------------
    Vector_CAM_FeatureValue         m_vectFeatureValue;
    CAM_FeatureDesc*                m_pFeatureDesc;
    CAM_FeatureDescFormat           m_stDescFormat;
    std::map<lx_uint32, lx_uint32>  m_mapFeatureIndex;

    inline void Free_Vector_CAM_FeatureValue(Vector_CAM_FeatureValue& vectFeatureValue)
    {
        if (vectFeatureValue.pstFeatureValue != NULL)
        {
            delete [] vectFeatureValue.pstFeatureValue;
            ZeroMemory(&vectFeatureValue, sizeof(Vector_CAM_FeatureValue));
        }
    }
    inline void Free_CAM_FeatureDesc()
    {
        if (m_pFeatureDesc != NULL)
        {
            delete [] m_pFeatureDesc;
            m_pFeatureDesc = NULL;
        }
    }

    NDArray *pRaw_;
    epicsEventId startEventId_;
};

extern "C" int KsCamConfig(char *portName, int cameraNumber, int maxBuffers,
                         size_t maxMemory, int priority, int stackSize)
{
    new KsCam(portName, cameraNumber, maxBuffers, maxMemory, priority, stackSize);
    return(asynSuccess);
}

static void imageGrabTaskC(void *drvPvt)
{
    KsCam *pPvt = (KsCam *)drvPvt;
    pPvt->imageGrabTask();
}


/** Constructor for KsCam driver; most parameters are simply passed to ADDriver::ADDriver.
  * After calling the base class constructor this method creates a thread to collect the detector data,
  * and sets reasonable default values for the parameters defined in this class, asynNDArrayDriver and ADDriver.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] cameraId The uniqueId of the camera to be connected to this driver. 0 if using an attached camera. If
  *            no cameras are attached then 0 is Ri2 simulator, and 1 is Qi2 simulator.  
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  */
KsCam::KsCam(const char *portName, int cameraNumber, int maxBuffers, size_t maxMemory,
         int priority, int stackSize)
    :ADDriver(portName, 1, NUM_KSCAM_PARAMS, maxBuffers, maxMemory, 0, 0, ASYN_CANBLOCK, 0, priority,
              stackSize),  pRaw_(NULL)
{
    int status = asynSuccess;
    static const char *functionName = "KsCam";
    m_pFeatureDesc = NULL;

    this->m_uiDeviceIndex=cameraNumber;
    this->m_uiCameraHandle=cameraNumber;
    this->m_uiDeviceCount=0;

    stImage.pDataBuffer = new BYTE[DEF_FRAME_SIZE_MAX];

    /* Exposure settings */
    createParam(ExposureModeString,         asynParamInt32,                &ExposureMode); 
    createParam(ExposureBiasString,         asynParamInt32,                &ExposureBias); 
    createParam(CaptureModeString,          asynParamInt32,                &CaptureMode); 
    createParam(FormatString,               asynParamInt32,                &Format); 
    createParam(ROIXString,                 asynParamInt32,                &ROIX); 
    createParam(ROIYString,                 asynParamInt32,                &ROIY); 

    /* Metering options */
    createParam(MeteringModeString,         asynParamInt32,                &MeteringMode);
    createParam(MeteringAreaLeftString,     asynParamInt32,                &MeteringAreaLeft);
    createParam(MeteringAreaTopString,      asynParamInt32,                &MeteringAreaTop);
    createParam(MeteringAreaWidthString,    asynParamInt32,                &MeteringAreaWidth);
    createParam(MeteringAreaHeightString,   asynParamInt32,                &MeteringAreaHeight);
    createParam(ExposureTimeLimitString,    asynParamFloat64,              &ExposureTimeLimit);
    createParam(GainLimitString,            asynParamInt32,                &GainLimit); 
    createParam(BrightnessString,           asynParamInt32,                &Brightness); 

    /* Ri2 only options relating to color */
    createParam(SharpnessString,            asynParamInt32,                &Sharpness); 
    createParam(HueString,                  asynParamInt32,                &Hue); 
    createParam(SaturationString,           asynParamInt32,                &Saturation); 
    createParam(WhiteBalanceRedString,      asynParamInt32,                &WhiteBalanceRed); 
    createParam(WhiteBalanceBlueString,     asynParamInt32,                &WhiteBalanceBlue); 
    createParam(ColorPresetString,          asynParamInt32,                &ColorPreset); 

    /* Triggering/Signaling options */
    createParam(NikonTriggerModeString,     asynParamInt32,                &NikonTriggerMode); 
    createParam(SignalExposureEndString,    asynParamInt32,                &SignalExposureEnd); 
    createParam(SignalTriggerReadyString,   asynParamInt32,                &SignalTriggerReady);
    createParam(SignalDeviceCaptureString,  asynParamInt32,                &SignalDeviceCapture);
    createParam(ExposureOutputString,       asynParamInt32,                &ExposureOutput);
    createParam(TriggerOptionFrameCtString, asynParamInt32,                &TriggerOptionFrameCt);
    createParam(TriggerOptionDelayString,   asynParamInt32,                &TriggerOptionDelay);

    /* Camera Commands */
    createParam(SoftTriggerString,          asynParamInt32,                &SoftTrigger); 
    createParam(TriggerCancelString,        asynParamInt32,                &TriggerCancel);
    createParam(FrameDroplessString,        asynParamInt32,                &FrameDropless);

    /* Camera Info */
    createParam(SignalEventString,          asynParamInt32,                &SignalEvent);
    createParam(NikonFrameRateString,       asynParamFloat64,              &NikonFrameRate);
    createParam(FwVersionString,            asynParamOctet,                &FwVersion);
    createParam(FpgaVersionString,          asynParamOctet,                &FpgaVersion); 
    createParam(Fx3VersionString,           asynParamOctet,                &Fx3Version); 
    createParam(UsbVersionString,           asynParamOctet,                &UsbVersion); 
    createParam(DriverVersionString,        asynParamOctet,                &DriverVersion); 
    createParam(SDKVersionString,           asynParamOctet,                &SDKVersion); 

    this->lock();
    status = connectCamera();
    this->unlock();
    if (status)
    {
        printf("%s:%s: cannot connect to camera, manually connect when available.\n",
               driverName, functionName);
        return;
    }

    /* This event will let the imageGrabTask thread know when to proceed */
    startEventId_ = epicsEventCreate(epicsEventEmpty);
    epicsThreadCreate("ImageAcquisitionTask",
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      imageGrabTaskC,this);

    /* Register the shutdown function for epicsAtExit */
    epicsAtExit(shutdown, (void*)this);
}

/* destructor */
KsCam::~KsCam()
{
    static const char *functionName = "~KsCam";
    this->lock();
    disconnectCamera();
    this->unlock();
}

void KsCam::shutdown (void* arg)
{
    KsCam *v = (KsCam*)arg;
    if (v) delete v;
}

/* Pointer for KsCam callback function to use */
KsCam* g_pDlg = NULL;

/* Camera event callback function */
FCAM_EventCallback fCAM_EventCallback(const lx_uint32 uiCameraHandle,
                                      CAM_Event* pstEvent, void* pTransData)
{
    g_pDlg->DoEvent(uiCameraHandle, pstEvent, pTransData);
    return 0;
}

/* Camera event callback function handler */
void KsCam::DoEvent(const lx_uint32 uiCameraHandle, CAM_Event* pstEvent, void* pTransData)
{
    static const char *functionName = "DoEvent";
    int nikonTriggerMode;
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,"%s:%s: Start Callback-DoEvent, EventNum=%i\n",
              driverName, functionName, pstEvent->eEventType);

    if ( uiCameraHandle != this->m_uiCameraHandle )
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,  "%s:%s: DoEvent Error, Invalid Camera Handle \n",
                  driverName, functionName);
        return;
    }

    switch(pstEvent->eEventType)
    {
        case    ecetImageReceived:
            /* Signal getImage thread to grab frame from camera */
            epicsEventSignal(this->startEventId_);
            FrameRate();
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s:%s: ImageRecieved, Frameno=%i, uiRemain=%i\n",
                      driverName, functionName, pstEvent->stImageReceived.uiFrameNo,
                      pstEvent->stImageReceived.uiRemained);
            break;
        case    ecetFeatureChanged:
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s:%s: FeatureChanged, Feature=%i\n",
                      driverName, functionName, m_mapFeatureIndex[pstEvent->stFeatureChanged.uiFeatureId]);
            /* Update the value that changed in m_vectFeatureValue */
            m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[pstEvent->stFeatureChanged.uiFeatureId]].stVariant
                = pstEvent->stFeatureChanged.stVariant;
            /* featureChanged will update the Description and set relevant params in the library */
            featureChanged(pstEvent->stFeatureChanged.uiFeatureId);
            break;
        case    ecetExposureEnd:
            setIntegerParam(SignalEvent, 0);
            setIntegerParam(ADStatus, ADStatusReadout);
            getIntegerParam(NikonTriggerMode, &nikonTriggerMode);
            /* Only close shutter if we are in soft/hard trigger modes */
            if ((nikonTriggerMode != 0))
            {  
                setShutter(0);
                /* If soft trigger mode, set back to zero so that SoftTrigger can be used as a scan variable */
                if (nikonTriggerMode == 2)
                    setIntegerParam(SoftTrigger, 0);
            }
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: ExposureEnd\n",
                      driverName, functionName);
            break;
        case    ecetTriggerReady:
            setIntegerParam(SignalEvent, 1);
            setIntegerParam(ADStatus, ADStatusIdle);
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: TriggerReady\n",
                      driverName, functionName);
            break;
        case    ecetDeviceCapture:
            setIntegerParam(SignalEvent, 2);
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: DeviceCapture\n",
                      driverName, functionName);
            break;
        case    ecetAeStay:
            setIntegerParam(SignalEvent, 3);
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: AeStay\n",
                      driverName, functionName);
            break;
        case    ecetAeRunning:
            setIntegerParam(SignalEvent, 4);
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: AeRunning\n",
                      driverName, functionName);
            break;
        case    ecetAeDisable:
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: AeDisable\n",
                      driverName, functionName);
            break;
        case    ecetTransError:
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: TransError T%u USB=0x%08lx  Driver=0x%08lx ReceivedSize=%d SettingSize=%d\n",
                      driverName, functionName, pstEvent->stTransError.uiTick,
                      pstEvent->stTransError.uiUsbErrorCode,
                      pstEvent->stTransError.uiDriverErrorCode, pstEvent->stTransError.uiReceivedSize,
                      pstEvent->stTransError.uiSettingSize);
            setIntegerParam(ADStatus, ADStatusError);
            break;
        case    ecetBusReset:
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: BusReset eBusResetCode:%i bImageCleared:%i\n",
                      driverName, functionName, pstEvent->stBusReset.eBusResetCode, pstEvent->stBusReset.bImageCleared );
            onBusReset(pstEvent->stBusReset.eBusResetCode, pstEvent->stBusReset.bImageCleared);
            break;
        default:
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Unknown Event %d\n",
                      driverName, functionName,  pstEvent->eEventType);
            break;
    }
    callParamCallbacks();
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,"%s:%s: End Callback-DoEvent, EventNum=%i\n",
              driverName, functionName, pstEvent->eEventType);
}

/* closely copied from kscam example */
void KsCam::onBusReset(lx_uint32 busReset, lx_uint32 imageCleared)
{
    static const char *functionName = "onBusReset";
    const char *info = "";

    switch(busReset) {
        case    ecebrcHappened:     
            info = "USB BUS was reset!";  
            setIntegerParam(ADStatus, ADStatusDisconnected);
            break;
        case    ecebrcRestored:
            GetAllFeaturesDesc(); 
            info = "USB BUS was restored!";
            setIntegerParam(ADStatus, ADStatusIdle);
            break;
        case    ecebrcFailed:       
            info = "USB BUS failed to restore!";
            setIntegerParam(ADStatus, ADStatusError);
            break;
        default:
            info = "Unknown";
            break;
    }
        
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: BusReset: %s, ImageCleared: %i\n",
              driverName, functionName, info, imageCleared);

    callParamCallbacks();
    return;
}

/* From asynPortDriver: Disconnects driver from device; */
asynStatus KsCam::disconnect( asynUser* pasynUser )
{
    return disconnectCamera();
}

asynStatus KsCam::disconnectCamera()
{
    int status = asynSuccess;
    static const char *functionName = "disconnectCamera";

    lx_result   lResult = LX_OK;

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: disconnecting camera \n",
              driverName, functionName);

    if ( this->m_isOpened )
    {
        lResult = CAM_Close(m_uiCameraHandle);
        if ( lResult != LX_OK )
        {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: Unable to close camera : %s \n",
                      driverName, functionName, KsErrorMap[lResult]);
            return asynError;
        }
        Free_Vector_CAM_FeatureValue(m_vectFeatureValue);
        Free_CAM_FeatureDesc();
        this->m_uiDeviceIndex = 0;
        this->m_uiCameraHandle = 0;
        this->m_isOpened = FALSE;
        g_pDlg = NULL;

        if (stImage.pDataBuffer != NULL)
        {
            delete [] stImage.pDataBuffer;
            stImage.pDataBuffer = NULL;
        }

    }

    /* We've disconnected the camera. Signal to asynManager that we are disconnected. */
    status = pasynManager->exceptionDisconnect(this->pasynUserSelf);
    if (status)
    {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: error calling pasynManager->exceptionDisconnect, error=%s\n",
                  driverName, functionName, pasynUserSelf->errorMessage);
    }

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: Camera disconnected\n",
              driverName, functionName);
    return asynSuccess;
}

/* From asynPortDriver: Connects driver to device; */
asynStatus KsCam::connect( asynUser* pasynUser )
{
    return connectCamera();
}

asynStatus KsCam::connectCamera()
{
    static const char *functionName = "connectCamera";
    int result = asynSuccess;

    lx_uint32               i;
    lx_result               lResult = LX_OK;
    lx_wchar                szError[CAM_ERRMSG_MAX];
    CAM_Device*             m_pstDevice;
    CAM_CMD_GetSdkVersion   stVersion;
    char                    *str = new char[CAM_NAME_MAX];

    ZeroMemory(szError, sizeof(szError));

    if ( m_uiDeviceCount > 0 )
    {
        lResult = CAM_CloseDevices();
        if (lResult != LX_OK )
        {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: Error calling CAM_CloseDevices(). %s \n",
                      driverName, functionName, KsErrorMap[lResult]);
            return asynError;
        }

        m_pstDevice = NULL;
        m_uiDeviceCount = 0;
    }

    lResult = CAM_OpenDevices(m_uiDeviceCount, &m_pstDevice);
    if ( lResult != LX_OK )
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Error calling CAM_OpenDevices(). [%s] Number of Devices [%i]\n",
                  driverName, functionName, KsErrorMap[lResult], m_uiDeviceCount);
        return asynError;
    }

    /* list devices detected */
    for( i=0; i<m_uiDeviceCount; i++ )
    {
        wcstombs(str, m_pstDevice[i].wszCameraName, 34);
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                  "%s:%s: CAM_OpenDevices(). [%s] Index: [%i]\n",
                  driverName, functionName, str, i);
    }

    lResult = CAM_Open(this->m_uiDeviceIndex, m_uiCameraHandle, countof(szError), szError);
    if ( lResult != LX_OK )
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Error calling CAM_Open(). [%s] Cam number [%i]\n",
                  driverName, functionName, szError, this->m_uiDeviceIndex);
        return asynError;
    }

    /* Connection was succesful, set Device info so we can use it later */
    this->m_stDevice = m_pstDevice[this->m_uiDeviceIndex];
    this->m_isOpened = TRUE; 

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: CAM_Open() Succesful. Cam number [%i]\n",
              driverName, functionName, this->m_uiDeviceIndex);

    lResult = CAM_SetEventCallback(m_uiCameraHandle, (FCAM_EventCallback)fCAM_EventCallback,
                                   m_ptrEventTransData);
    if ( lResult != LX_OK )
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Error calling CAM_SetEventCallback(). %s \n",
                  driverName, functionName, KsErrorMap[lResult]);
        return asynError;
    }
    
    /* Need this to setup callback access */
    g_pDlg = this;

    /* Get all feature values and descriptions and update relevant records */
    GetAllFeatures();
    GetAllFeaturesDesc();    
    
    /* Get Dropless Frame status */
    GetDropless(); 

    /* Print all features and their settings for reference */
    PrintAllFeatures();

    /* All of these are constant and thus can be set once at connection and not touched again */
    result  = setStringParam(ADManufacturer, "Nikon");
    wcstombs(str,this->m_stDevice.wszCameraName,CAM_NAME_MAX);
    result |= setStringParam(ADModel, str);
    wcstombs(str,this->m_stDevice.wszFwVersion,CAM_VERSION_MAX);
    result |= setStringParam(FwVersion, str);
    wcstombs(str,this->m_stDevice.wszFpgaVersion,CAM_VERSION_MAX);
    result |= setStringParam(FpgaVersion, str);
    wcstombs(str,this->m_stDevice.wszFx3Version,CAM_VERSION_MAX);
    result |= setStringParam(Fx3Version, str);
    wcstombs(str,this->m_stDevice.wszUsbVersion,CAM_VERSION_MAX);
    result |= setStringParam(UsbVersion, str);
    wcstombs(str,this->m_stDevice.wszDriverVersion,CAM_VERSION_MAX);
    result |= setStringParam(DriverVersion, str);
    result |= setIntegerParam(ADMaxSizeX, 4908); //true for both Qi2 and Ri2
    result |= setIntegerParam(ADMaxSizeY, 3264); //true for both Qi2 and Ri2

    lResult = CAM_Command(m_uiCameraHandle, CAM_CMD_GET_SDKVERSION, &stVersion);
    if (lResult != LX_OK)
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Get SDK Version Error %s \n", driverName, functionName,
                  KsErrorMap[lResult]);
    }
    wcstombs(str,stVersion.wszSdkVersion,CAM_VERSION_MAX);
    result |= setStringParam(SDKVersion, str);

    result = pasynManager->exceptionConnect(this->pasynUserSelf);
    if (result)
    {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: error calling pasynManager->exceptionConnect, error=%s\n",
                  driverName, functionName, pasynUserSelf->errorMessage);
        return asynError;
    }

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Camera succesfully connected\n",
              driverName, functionName);
    return asynSuccess;
}

void KsCam::imageGrabTask()
{
    static const char *functionName = "imageGrabTask";
    asynStatus status = asynSuccess;
    int numImages, numImagesCounter, imageCounter;
    int imageMode;
    int arrayCallbacks;
    epicsTimeStamp startTime;


    while (1)
    {
        /* Wait for a signal that tells this thread that the transmission
         * has started and we can start asking for image buffers.
           Signal is given by a camera callback indicating a frame has arrived.
           This is different from how other areadetector drivers work.    
           It seemed cleaner to do it this way since this camera has a callback feature
           to indicate frame arrival */
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s waiting for acquire to start\n",
                  driverName, functionName);
        
        /* Release the lock while we wait for an event that says acquire has started, then lock again */
        unlock();
        epicsEventWait(startEventId_);
        lock();
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,"%s::%s started!\n", driverName, functionName);

        /* Get the current time */
        epicsTimeGetCurrent(&startTime);

        status=grabImage();
        if (status == asynError)
        {
            /* remember to release the NDArray back to the pool now
             * that we are not using it (we didn't get an image...) */
            if (pRaw_) pRaw_->release();
            pRaw_ = NULL;
            continue;
        }

        getIntegerParam(NDArrayCounter, &imageCounter);
        getIntegerParam(ADNumImages, &numImages);
        getIntegerParam(ADNumImagesCounter, &numImagesCounter);
        getIntegerParam(ADImageMode, &imageMode);
        getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
        imageCounter++;
        numImagesCounter++;
        setIntegerParam(NDArrayCounter, imageCounter);
        setIntegerParam(ADNumImagesCounter, numImagesCounter);

        if (arrayCallbacks)
        {
            /* Call the NDArray callback */
            /* Must release the lock here, or we can get into a deadlock, because we can
             * block on the plugin lock, and the plugin can be calling us */
            unlock();
            doCallbacksGenericPointer(pRaw_, NDArrayData, 0);
            lock();
        }
        /* Release the NDArray buffer now that we are done with it.
         * After the callback just above we don't need it anymore */
        pRaw_->release();
        pRaw_ = NULL;

        /* See if acquisition is done if we are in single or multiple mode */
        if ((imageMode == ADImageSingle) || ((imageMode == ADImageMultiple)
                                             && (numImagesCounter >= numImages)))
        {
            status = stopCapture();
        }
        setIntegerParam(ADStatus, ADStatusIdle);
        callParamCallbacks();
    }
}

/* Called from imageGrabTask whenever the ImageRecieved event happens, grabs a single frame
   and passes it to the NDArray pool */
asynStatus KsCam::grabImage()
{
    static const char *functionName = "grabImage";
    asynStatus status = asynSuccess;
    lx_result           lResult = LX_OK;
    lx_uint32           uiRemained;
    CString             strText;
    CAM_ImageInfoEx     stImageInfoEx;
    CAM_ImageInfo*      pstInfo;
    NDDataType_t dataType;
    int nDims;
    size_t dims[3];
    size_t dataSize;
    void *pData;

    /* Figure out the framesize, we need it to use the GetImage command*/
    lResult = CAM_Command(m_uiCameraHandle, CAM_CMD_GET_FRAMESIZE, &m_stFrameSize);
    if ( lResult != LX_OK )
    {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s GetFrameSize Error. %s \n",
                  driverName, functionName, KsErrorMap[lResult]);
        return(asynError);
    }
    stImage.uiDataBufferSize = this->m_stFrameSize.uiFrameSize;

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s GetFrame Size interval=%d shutter delay=%d\n", driverName, functionName, 
              this->m_stFrameSize.uiFrameInterval,this->m_stFrameSize.uiRShutterDelay );

    /* Grab the Image */
    lResult = CAM_GetImage(m_uiCameraHandle, true, stImage, uiRemained);
    if ( lResult != LX_OK )
    {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s CAM_GetImage error. %s\n",
                  driverName, functionName, KsErrorMap[lResult]);
        return(asynError);
    }

    dataSize=stImage.uiImageSize;
    setIntegerParam(NDArraySize, (int)dataSize);
    pstInfo=stImageInfoEx.GetInfo(stImage);

    if (pstInfo->ucImageColor == ecfcMono16)
    {
        dataType= NDUInt16;
        nDims   = 2;
        dims[0] = pstInfo->usImageWidth;
        dims[1] = pstInfo->usImageHeight;
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
              "%s::%s Image info dim0=%i dim1=%i size=%i frameno=%i uiRemained=%i\n",
              driverName, functionName, dims[0], dims[1], dataSize, pstInfo->usFrameNo, uiRemained);
    } else {
        dataType= NDUInt8;
        nDims   = 3;
        dims[0] = 3;
        dims[1] = pstInfo->usImageWidth;
        dims[2] = pstInfo->usImageHeight;
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
              "%s::%s Image info dim0=%i dim1=%i dim2=%i size=%i frameno=%i uiRemained=%i\n",
              driverName, functionName, dims[0], dims[1], dims[2], dataSize, pstInfo->usFrameNo, uiRemained);
    }

    pRaw_ = pNDArrayPool->alloc(nDims, dims, dataType, 0, NULL);
    if (!pRaw_)
    {
        /* If we didn't get a valid buffer from the NDArrayPool we must abort
         * the acquisition as we have nowhere to dump the data...       */
        setIntegerParam(ADStatus, ADStatusAborting);
        callParamCallbacks();
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s ERROR: Serious problem: not enough buffers left! Aborting acquisition!\n",
                  driverName, functionName);
        setIntegerParam(ADAcquire, 0);
        return(asynError);
    }
    pData = stImage.pDataBuffer;
    memcpy(pRaw_->pData, pData, dataSize);

    /* Put the frame number into the buffer */
    pRaw_->uniqueId = pstInfo->usFrameNo;
    updateTimeStamp(&pRaw_->epicsTS);
    pRaw_->timeStamp = pRaw_->epicsTS.secPastEpoch + pRaw_->epicsTS.nsec/1e9;

    getAttributes(pRaw_->pAttributeList);

    callParamCallbacks();
    return(asynSuccess);
}

asynStatus KsCam::startCapture()
{
    static const char *functionName = "startCapture";
    int nikonTriggerMode;
    /* Start the camera transmission... */
    setIntegerParam(ADAcquire, 1);
    setIntegerParam(ADNumImagesCounter, 0);
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
              "%s::%s calling CameraBase::StartCapture\n",
              driverName, functionName);
    getIntegerParam(NikonTriggerMode, &nikonTriggerMode);
    /* Only open shutter if we are not in soft or hard trigger modes
       In soft trigger mode, soft trigger command will open shutter. 
       In hard trigger mode, shutter control will have to be external. */
    if ((nikonTriggerMode == 0))
        setShutter(1);
    Command(CAM_CMD_START_FRAMETRANSFER);
    return asynSuccess;
}

asynStatus KsCam::stopCapture()
{
    static const char *functionName = "stopCapture";
    setShutter(0);
    Command(CAM_CMD_STOP_FRAMETRANSFER);
    setIntegerParam(ADAcquire, 0);
    return asynSuccess;
}

/* Updates EPICS param for a given uiFeatureId */
void KsCam::featureChanged(lx_uint32 uiFeatureId)
{
    static const char *functionName = "featureChanged";
    lx_result       lResult = LX_OK;
    lx_uint32       uiIndex;

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                  "%s::%s Start function uiFeatureId=%i\n",
                  driverName, functionName, uiFeatureId);

    uiIndex = m_mapFeatureIndex[uiFeatureId];
    
    /* First update FeatureDesc because it may have changed */
    lResult = CAM_GetFeatureDesc(m_uiCameraHandle, uiFeatureId, m_pFeatureDesc[uiIndex]);
    if ( lResult != LX_OK ) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Error Calling CAM_GetFeatureDesc uiFeatureId=%i %s \n",
                  driverName, functionName, uiFeatureId, KsErrorMap[lResult]);
    }
    
    /* Update relevant EPICS params, note that for a given "feature" there may be more than one value to update */
    switch(uiFeatureId)
    {
        case    eExposureMode:
            setIntegerParam(ExposureMode,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eExposureMode]].stVariant.ui32Value);
            break;
        case    eExposureBias:
            setIntegerParam(ExposureBias,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eExposureBias]].stVariant.i32Value);
            break;
        case    eExposureTime:
            setDoubleParam(ADAcquireTime,
                       m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eExposureTime]].stVariant.ui32Value/1000000.);
            break;
        case    eGain:
            setDoubleParam(ADGain,
                       m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eGain]].stVariant.ui32Value);
            break;
        case    eMeteringMode:
            setIntegerParam(MeteringMode,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eMeteringMode]].stVariant.ui32Value);
            break;
        case    eMeteringArea:
            setIntegerParam(MeteringAreaLeft,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eMeteringArea]].stVariant.stArea.uiLeft);
            setIntegerParam(MeteringAreaTop,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eMeteringArea]].stVariant.stArea.uiTop);
            setIntegerParam(MeteringAreaWidth,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eMeteringArea]].stVariant.stArea.uiWidth);
            setIntegerParam(MeteringAreaHeight,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eMeteringArea]].stVariant.stArea.uiHeight);
            break;
        case    eExposureTimeLimit:
            setDoubleParam(ExposureTimeLimit,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eExposureTimeLimit]].stVariant.ui32Value/1000000.);
            break;
        case    eGainLimit:
            setIntegerParam(GainLimit,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eGainLimit]].stVariant.ui32Value);
            break;
        case    eCaptureMode:
            setIntegerParam(CaptureMode,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eCaptureMode]].stVariant.ui32Value);
            break;
        case    eBrightness:
            setIntegerParam(Brightness,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eBrightness]].stVariant.i32Value);
            break;
        case    eSharpness:
            setIntegerParam(Sharpness,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eSharpness]].stVariant.i32Value);
            break;
        case    eHue:
            setIntegerParam(Hue,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eHue]].stVariant.i32Value);
            break;
        case    eSaturation:
            setIntegerParam(Saturation,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eSaturation]].stVariant.i32Value);
            break;
        case    eWhiteBalanceRed:
            setIntegerParam(WhiteBalanceRed,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eWhiteBalanceRed]].stVariant.i32Value);
            break;
        case    eWhiteBalanceBlue:
            setIntegerParam(WhiteBalanceBlue,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eWhiteBalanceBlue]].stVariant.i32Value);
            break;
        case    ePresets:
            setIntegerParam(ColorPreset,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[ePresets]].stVariant.i32Value);
            break;
        case    eTriggerOption:
            setIntegerParam(TriggerOptionFrameCt,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eTriggerOption]].stVariant.stTriggerOption.uiFrameCount);
            setIntegerParam(TriggerOptionDelay,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eTriggerOption]].stVariant.stTriggerOption.iDelayTime);
            break;
        case    eMultiExposureTime:
            /* not currently implemented */
            break;
        case    eSignalExposureEnd:
            setIntegerParam(SignalExposureEnd,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eSignalExposureEnd]].stVariant.ui32Value);
            break;
        case    eSignalTriggerReady:
            setIntegerParam(SignalTriggerReady,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eSignalTriggerReady]].stVariant.ui32Value);
            break;
        case    eSignalDeviceCapture:
            setIntegerParam(SignalDeviceCapture,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eSignalDeviceCapture]].stVariant.ui32Value);
            break;
        case    eExposureOutput:
            setIntegerParam(ExposureOutput, 
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eExposureOutput]].stVariant.ui32Value);
            break;
        case    eFormat:
            updateImageSettings();
            break;
        case    eRoiPosition:
            setIntegerParam(ROIX,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eRoiPosition]].stVariant.stPosition.uiX);
            setIntegerParam(ROIY,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eRoiPosition]].stVariant.stPosition.uiY);
            break;
        case    eTriggerMode:
            setIntegerParam(NikonTriggerMode,
                        m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eTriggerMode]].stVariant.ui32Value);
            break;
        default:
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s::%s Warning, Unrecognized FeatureId uiFeatureId=%i\n",
                      driverName, functionName, uiFeatureId);
            break;
    }

    callParamCallbacks();
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                  "%s::%s End function uiFeatureId=%i\n",
                  driverName, functionName, uiFeatureId);
}

/* Updates all EPICS params relating to image settings */
asynStatus KsCam::updateImageSettings()
{
    static const char *functionName = "updateImageSettings";
    int status = asynSuccess;

    lx_uint32   formatIndex;
    CString     strName;
    CString     strValue;
    lx_result lResult = LX_OK;
    char *str = new char[CAM_NAME_MAX];

    switch(m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eFormat]].stVariant.stFormat.eColor)
    {
    case ecfcUnknown:
        setIntegerParam(NDColorMode, NDColorModeMono );
        setIntegerParam(NDDataType,  NDUInt16);
        break;
    case ecfcRgb24:
        setIntegerParam(NDColorMode, NDColorModeRGB1 );
        break;
    case ecfcYuv444:
        setIntegerParam(NDColorMode, NDColorModeYUV444 );
        break;
    case ecfcMono16:
        setIntegerParam(NDColorMode, NDColorModeMono );
        setIntegerParam(NDDataType,  NDUInt16);
        break;
    }

    /* The following determines which image format is currently set. Unfortunately this is much more complicated than it should be but the format is split 
       into two substructures (eMode and eColor), and apparently you can't set those values individually */
    CAM_FeatureDesc* pFeatureDesc = &m_pFeatureDesc[m_mapFeatureIndex[eFormat]];
    for (formatIndex = 0; formatIndex < pFeatureDesc->uiListCount; formatIndex++)
    {
        if (pFeatureDesc->stFormatList[formatIndex].stFormat ==
                m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eFormat]].stVariant.stFormat)
        {
            setIntegerParam(Format,  formatIndex);
            break;
        }
    }

    switch(m_vectFeatureValue.pstFeatureValue[m_mapFeatureIndex[eFormat]].stVariant.stFormat.eMode)
    {
    case ecfmUnknown:
        setIntegerParam(NDArraySizeX, 0);
        setIntegerParam(NDArraySizeY, 0);
        break;
    case ecfm4908x3264:
        setIntegerParam(NDArraySizeX, 4908);
        setIntegerParam(NDArraySizeY, 3264);
        setIntegerParam(ADSizeX, 4908);
        setIntegerParam(ADSizeY, 3264);
        break;
    case ecfm2454x1632:
        setIntegerParam(NDArraySizeX, 2454);
        setIntegerParam(NDArraySizeY, 1632);
        setIntegerParam(ADSizeX, 2454);
        setIntegerParam(ADSizeY, 1632);        
        break;
    case ecfm1636x1088:
        setIntegerParam(NDArraySizeX, 1636);
        setIntegerParam(NDArraySizeY, 1088);
        setIntegerParam(ADSizeX, 1636);
        setIntegerParam(ADSizeY, 1088);
        break;
    case ecfm818x544:
        setIntegerParam(NDArraySizeX, 818);
        setIntegerParam(NDArraySizeY, 544);
        setIntegerParam(ADSizeX, 818);
        setIntegerParam(ADSizeY, 544);
        break;
    case ecfm1608x1608:
        setIntegerParam(NDArraySizeX, 1608);
        setIntegerParam(NDArraySizeY, 1608);
        setIntegerParam(ADSizeX, 1608);
        setIntegerParam(ADSizeY, 1608);
        break;
    case ecfm804x804:
        setIntegerParam(NDArraySizeX, 804);
        setIntegerParam(NDArraySizeY, 804);
        setIntegerParam(ADSizeX, 804);
        setIntegerParam(ADSizeY, 804);
        break;
    case ecfm536x536:
        setIntegerParam(NDArraySizeX, 536);
        setIntegerParam(NDArraySizeY, 536);
        setIntegerParam(ADSizeX, 536);
        setIntegerParam(ADSizeY, 536);
        break;
    default:
        setIntegerParam(NDArraySizeX, 0);
        setIntegerParam(NDArraySizeY, 0);
        break;
    }

    lResult = CAM_Command(m_uiCameraHandle, CAM_CMD_GET_FRAMESIZE, &m_stFrameSize);
    if ( lResult != LX_OK )
    {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s GetFrameSize Error. %s\n",
                  driverName, functionName, KsErrorMap[lResult]);
        return asynError;
    }
    setIntegerParam(NDArraySize, this->m_stFrameSize.uiFrameSize);

    callParamCallbacks();

    if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: error, status=%d\n",
                              driverName, functionName, status);
    return((asynStatus)status);
}

void KsCam::GetAllFeatures()
{
    static const char *functionName = "getAllFeatures";

    int result = asynSuccess;
    lx_result   lResult = LX_OK;

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: GetAllFeatures() Start\n",
              driverName, functionName);

    Free_Vector_CAM_FeatureValue(m_vectFeatureValue);

    m_vectFeatureValue.uiCapacity = CAM_FEA_CAPACITY;
    m_vectFeatureValue.pstFeatureValue = new
    CAM_FeatureValue[m_vectFeatureValue.uiCapacity];

    if ( !m_vectFeatureValue.pstFeatureValue )
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: GetAllFeatures() Memory allocation error. \n",
                  driverName, functionName);
        return;
    }

    lResult = CAM_GetAllFeatures(m_uiCameraHandle, m_vectFeatureValue);

    if ( lResult != LX_OK )
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: GetAllFeatures() error. %s \n",
                  driverName, functionName, KsErrorMap[lResult]);
        return;
    }

    if ( m_vectFeatureValue.uiCountUsed == 0 )
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Error: GetAllFeatures() returned no features.\n",
                  driverName, functionName);
    }

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: GetAllFeatures() End\n",
              driverName, functionName);
}

/* This function creates the feature map and calls featureChanged for every feature */
void KsCam::GetAllFeaturesDesc()
{
    static const char *functionName = "GetAllFEaturesDesc";

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: GetAllFeatures start().\n",
              driverName, functionName );

    lx_result   lReturn = LX_OK;
    lx_uint32   uiFeatureId, i;
    CString     strValue;

    Free_CAM_FeatureDesc();
    m_mapFeatureIndex.clear();

    m_pFeatureDesc = new CAM_FeatureDesc[m_vectFeatureValue.uiCountUsed];
    if ( !m_pFeatureDesc )
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: GetAllFeaturesDesc memory allocate Error.[] \n",
                  driverName, functionName );
        return;
    }

    /* This loops through the total number of features on the given device */
    for( i=0; i<m_vectFeatureValue.uiCountUsed; i++ )
    {
        uiFeatureId = m_vectFeatureValue.pstFeatureValue[i].uiFeatureId;
        /* map the FeatureId to i */
        m_mapFeatureIndex.insert(std::make_pair(uiFeatureId, i));        
        /* featureChanged will update feature description and set relevant EPICS param values */
        featureChanged(uiFeatureId);
    }

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: GetAllFeaturesDesc end(). [] \n",
              driverName, functionName );
}

/* This function prints all supported features and their current settings */
void KsCam::PrintAllFeatures()
{
    static const char *functionName = "PrintAllFeatures";
    lx_uint32   i;
    CString     strName;
    CString     strValue;
    lx_uint32   uiIndex;

    for (i = 0; i < m_vectFeatureValue.uiCountUsed; i++)
    {
        strName = ConvFeatureIdToName(m_vectFeatureValue.pstFeatureValue[i].uiFeatureId);
        uiIndex = m_mapFeatureIndex[m_vectFeatureValue.pstFeatureValue[i].uiFeatureId];
        strValue = GetFeatureDispString(&m_vectFeatureValue.pstFeatureValue[uiIndex]);
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                  "%s:%s:Featurelist: %s %s uiIndex[%d] uiFeatureId [%d] \n",
                  driverName, functionName, strName, strValue, uiIndex,
                  m_vectFeatureValue.pstFeatureValue[i].uiFeatureId);
    }
}

/* After a new value is set to vectFeatureValue by SetIntValue or SetFloatValue this calls 
   CAM_SetFeature to set the value to the camera. If command is succesfully it calls 
   featureChanged for the relevant uiFeatureId */
asynStatus KsCam::SetFeature(lx_uint32 uiFeatureId)
{
    static const char *functionName = "SetFeature";

    lx_result                   lResult = LX_OK;
    lx_uint32                   uiIndex;
    CString                     strValue, strName;
    Vector_CAM_FeatureValue     vectFeatureValue;

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: SetFeature(%i) start\n",
              driverName, functionName, uiFeatureId);

    /* Prepare the vectFeatureValue structure to use in the CAM_setFeatures command */
    vectFeatureValue.uiCountUsed = 1;
    vectFeatureValue.uiCapacity = 1;
    vectFeatureValue.uiPauseTransfer = 0;
    vectFeatureValue.pstFeatureValue = new CAM_FeatureValue[1];
    if (vectFeatureValue.pstFeatureValue == NULL)
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Error allocating memory vecFeatureValue. \n",
                  driverName, functionName );
        return asynError;
    }

    uiIndex = m_mapFeatureIndex[uiFeatureId];
    vectFeatureValue.pstFeatureValue[0] = m_vectFeatureValue.pstFeatureValue[uiIndex];

    lResult = CAM_SetFeatures(m_uiCameraHandle, vectFeatureValue);
    Free_Vector_CAM_FeatureValue(vectFeatureValue);
    if ( lResult != LX_OK )
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: CAM_SetFeatures Error (%s) %s \n", driverName, functionName, 
                  ConvFeatureIdToName(uiFeatureId), KsErrorMap[lResult]);
        return asynError;
    }

    /* only update Param value if it was a success */
    featureChanged(uiFeatureId);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: SetFeature(%i) end\n",
              driverName, functionName,  uiFeatureId);
    
    return asynSuccess;
}

/* Function to get the FrameDropless setting. Does not work with simulators. */
void KsCam::GetDropless()
{
    static const char *functionName = "GetDropless";
    lx_result   lResult = LX_OK;

    /* Set up structure to pass to camera */
    CAM_CMD_FrameDropless      stDrop;
    stDrop.bSet = FALSE;

    lResult = CAM_Command(m_uiCameraHandle, CAM_CMD_FRAME_DROPLESS, &stDrop);
    if (lResult != LX_OK)
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Error. %s \n", driverName, functionName,
                  KsErrorMap[lResult]);
        return;
    }
    setIntegerParam(FrameDropless, stDrop.bOnOff);
    return;
}

/* Sends a single command to the device, and depending on what command it is takes different actions */
void KsCam::Command(const lx_wchar* wszCommand)
{
    static const char *functionName = "Command";
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Start (%s) \n", driverName,
              functionName, wszCommand);

    lx_result       lResult = LX_OK;

    if ( !_wcsicmp(wszCommand, CAM_CMD_START_FRAMETRANSFER) )
    {
        CAM_CMD_StartFrameTransfer      stCmd;

        /* Start frame transfer */
        stCmd.uiImageBufferNum = DEF_DRIVER_BUFFER_NUM;
        lResult = CAM_Command(m_uiCameraHandle, CAM_CMD_START_FRAMETRANSFER, &stCmd);
        if (lResult != LX_OK)
        {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: (%s) Error. %s \n", driverName, functionName, wszCommand,
                      KsErrorMap[lResult]);
            return;
        }

        /* Frame rate calc start */
        FrameRateStart();
    }
    else if ( !_wcsicmp(wszCommand, CAM_CMD_FRAME_DROPLESS) )
    {
        /* Toggle dropless mode */
        CAM_CMD_FrameDropless      stDrop;
        stDrop.bSet = TRUE;

        int dropless;
        getIntegerParam(FrameDropless, &dropless);

        if (dropless == TRUE)
        {
            stDrop.bOnOff = FALSE;
        }
        else
        {
            stDrop.bOnOff = TRUE;
        }

        lResult = CAM_Command(m_uiCameraHandle, CAM_CMD_FRAME_DROPLESS, &stDrop);
        if (lResult != LX_OK)
        {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: (%s) Error. %s \n", driverName, functionName, wszCommand,
                      KsErrorMap[lResult]);
            return;
        }

        /* verify with readback value */
        GetDropless();
    }
    else
    {
        lResult = CAM_Command(m_uiCameraHandle, wszCommand, NULL);
        if (lResult != LX_OK)
        {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: (%s) Error. %s \n", driverName, functionName, wszCommand,
                      KsErrorMap[lResult]);
            return;
        }
    }

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: End (%s) \n", driverName,
              functionName, wszCommand);
}


/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters, including ADAcquire, ADBinX, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
/* Most epics params only differ from the FeatureId in that the FeatureId has an e prepended to 
   the setting name. This could probably be used to turn this lengthy chain of if/else statements in to 
   only a few statements. However I do not know of a clean/simple way to associate each FeatureId
   with another an EPICS parameter while also mantaining an iterable list of uiFeatureId's which 
   is currently setup in the GetAllFeatures function */
asynStatus KsCam::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    static const char *functionName = "writeInt32";
    int function = pasynUser->reason;
    int acquire;
    asynStatus status = asynSuccess;

    //status = setIntegerParam(function, value);

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: writeInt32, status=%d function=%d, value=%d\n", driverName, functionName,
              status, function, value);

    if (function == ADAcquire)
    {
        if (value)
        {
            setIntegerParam(ADStatus, ADStatusWaiting);
            status = startCapture();
        }
        else
        {
            setIntegerParam(ADStatus, ADStatusIdle);
            status = stopCapture();
        }
    }
    else if (function == ExposureMode)
    {
        status = SetIntValue(eExposureMode, value, 0);
    }
    else if (function == ExposureBias)
    {
        status = SetIntValue(eExposureBias, value, 0);
    }
    else if (function == Format)
    {
        status = SetIntValue(eFormat, value, 0);
    }
    else if (function == CaptureMode)
    {
        status = SetIntValue(eCaptureMode, value, 0);
    }
    else if (function == NikonTriggerMode)
    {
        status = SetIntValue(eTriggerMode, value, 0);
    }
    else if (function == SignalExposureEnd)
    {
        status = SetIntValue(eSignalExposureEnd, value, 0);
    }
    else if (function == SignalTriggerReady)
    {
        status = SetIntValue(eSignalTriggerReady, value, 0);
    }
    else if (function == SignalDeviceCapture)
    {
        status = SetIntValue(eSignalDeviceCapture, value, 0);
    }
    else if (function == ExposureOutput)
    {
        status = SetIntValue(eExposureOutput, value, 0);
    }
    else if (function == Brightness)
    {
        status = SetIntValue(eBrightness, value, 0);
    }
    else if (function == MeteringMode)
    {
        status = SetIntValue(eMeteringMode, value, 0);
    }
    else if (function == GainLimit)
    {
        status = SetIntValue(eGainLimit, value, 0);
    }
    else if (function == MeteringAreaLeft)
    {
        status = SetIntValue(eMeteringArea, value, 0);
    }
    else if (function == MeteringAreaTop)
    {
        status = SetIntValue(eMeteringArea, value, 1);
    }
    else if (function == MeteringAreaWidth)
    {
        status = SetIntValue(eMeteringArea, value, 2);
    }
    else if (function == MeteringAreaHeight)
    {
        status = SetIntValue(eMeteringArea, value, 3);
    }
    else if (function == TriggerOptionDelay)
    {
        status = SetIntValue(eTriggerOption, value, 0);
    }
    else if (function == TriggerOptionFrameCt)
    {
        status = SetIntValue(eTriggerOption, value, 1);
    }
    else if (function == ROIX)
    {
        status = SetIntValue(eRoiPosition, value, 0);
    }
    else if (function == ROIY)
    {
        status = SetIntValue(eRoiPosition, value, 1);
    }
    else if (function == SoftTrigger)
    {   
        /* Only allow SoftTrigger command if acquire is active */
        getIntegerParam(ADAcquire, &acquire);
        if ( acquire )
        {
            setIntegerParam(ADStatus, ADStatusWaiting);
            setShutter(1);
            setIntegerParam(SoftTrigger, 1);
            Command(CAM_CMD_ONEPUSH_SOFTTRIGGER);
        }
        else 
        {
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: error, SoftTrigger requires Acquire to be enabled \n",
                  driverName, functionName);
        }
    }
    else if (function == TriggerCancel)
    {
        Command(CAM_CMD_ONEPUSH_TRIGGERCANCEL);
    }
    else if (function == FrameDropless)
    {
        Command(CAM_CMD_FRAME_DROPLESS);
    }
    else if (function == Sharpness)
    {
        status = SetIntValue(eSharpness, value, 0);
    }
    else if (function == Hue)
    {
        status = SetIntValue(eHue, value, 0);
    }
    else if (function == Saturation)
    {
        status = SetIntValue(eSaturation, value, 0);
    }
    else if (function == WhiteBalanceRed)
    {
        status = SetIntValue(eWhiteBalanceRed, value, 0);
    }
    else if (function == WhiteBalanceBlue)
    {
        status = SetIntValue(eWhiteBalanceRed, value, 0);
    }
    else if (function == ColorPreset)
    {
        status = SetIntValue(ePresets, value, 0);
    }        

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: error, status=%d function=%d, value=%d\n",
                  driverName, functionName, status, function, value);
    else{
        status = setIntegerParam(function, value);
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                  "%s:%s: function=%d, value=%d\n",
                  driverName, functionName, function, value);
    }
    callParamCallbacks();
    return((asynStatus)status);
}

/* Called from writeInt32 for a given uiFeatureId. Because a given uiFeatureId may have
   multiple Parameters associated with it the subFeature value is used to distinguish them. 
   There is likely a much more elegant way to do this, but I have yet to think of one that 
   doesn't result in unnecessary added complexity */
asynStatus KsCam::SetIntValue(lx_uint32 uiFeatureId, epicsInt32 value, int subFeature)
{
    static const char *functionName = "SetIntValue";
    asynStatus status = asynSuccess;
    CAM_FeatureValue*   pFeatureValue;
    CAM_FeatureDesc*    pFeatureDesc;
    lx_uint32                uiIndex;

    uiIndex       = m_mapFeatureIndex[uiFeatureId];
    pFeatureValue = &m_vectFeatureValue.pstFeatureValue[uiIndex];
    pFeatureDesc  = &m_pFeatureDesc[uiIndex];

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Start eVarType: %i\n",
              driverName, functionName, pFeatureValue->stVariant.eVarType);

    switch( pFeatureDesc->eFeatureDescType )
    {
        case    edesc_ElementList:
            pFeatureValue->stVariant.i32Value = value;
            break;
        case    edesc_Range:
            switch (pFeatureValue->stVariant.eVarType)
            {
                case    evrt_int32:
                    if ( !IsInRange(value, pFeatureDesc->stRange) )
                    {
                        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                                      "%s:%s: OutofRange error, function=%d, value=%d min=%d max=%d res=%d\n",
                                      driverName, functionName, uiFeatureId, value,
                                      pFeatureDesc->stRange.stMin.i32Value,
                                      pFeatureDesc->stRange.stMax.i32Value,
                                      pFeatureDesc->stRange.stRes.i32Value);
                        return asynError;
                    }
                    pFeatureValue->stVariant.i32Value = value;
                    break;
                case    evrt_uint32:
                    if ( !IsInRangeU(value, pFeatureDesc->stRange) )
                    {
                        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                                  "%s:%s: OutofRange error, function=%d, value=%d\n",
                                  driverName, functionName, uiFeatureId, value);
                        return asynError;
                    }
                    pFeatureValue->stVariant.ui32Value = value;
                    break;
            }
            break;
        case    edesc_FormatList:
            pFeatureValue->stVariant.stFormat = pFeatureDesc->stFormatList[value].stFormat;
            break;
        case    edesc_Area:
            if ( subFeature == 0)
            {
                if ( !IsInRange(value, pFeatureDesc->stArea.stMin.uiLeft,
                                pFeatureDesc->stArea.stMax.uiLeft, pFeatureDesc->stArea.stRes.uiLeft) )
                {
                    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: OutofRange error, function=%d, value=%d\n",
                              driverName, functionName, uiFeatureId, value);
                    return asynError;
                }
                pFeatureValue->stVariant.stArea.uiLeft = value;
                break;
            }
            if ( subFeature == 1)
            {
                if ( !IsInRange(value,  pFeatureDesc->stArea.stMin.uiTop,
                                pFeatureDesc->stArea.stMax.uiTop, pFeatureDesc->stArea.stRes.uiTop) )
                {
                    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: OutofRange error, function=%d, value=%d\n",
                              driverName, functionName, uiFeatureId, value);
                    return asynError;
                }
                pFeatureValue->stVariant.stArea.uiTop = value;
                break;
            }
            if ( subFeature == 2)
            {
                if ( !IsInRange(value,  pFeatureDesc->stArea.stMin.uiWidth,
                                pFeatureDesc->stArea.stMax.uiWidth, pFeatureDesc->stArea.stRes.uiWidth) )
                {
                    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: OutofRange error, function=%d, value=%d\n",
                              driverName, functionName, uiFeatureId, value);
                    return asynError;
                }
                pFeatureValue->stVariant.stArea.uiWidth = value;
                break;
            }
            if ( subFeature == 3)
            {
                if ( !IsInRange(value,  pFeatureDesc->stArea.stMin.uiHeight,
                                pFeatureDesc->stArea.stMax.uiHeight, pFeatureDesc->stArea.stRes.uiHeight) )
                {
                    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: OutofRange error, function=%d, value=%d\n",
                              driverName, functionName, uiFeatureId, value);
                    return asynError;
                }
                pFeatureValue->stVariant.stArea.uiHeight = value;
                break;
            }
        case    edesc_TriggerOption:
            if ( subFeature == 0)
            {
                if ( !IsInRange(value, pFeatureDesc->stTriggerOption.stRangeDelayTime) )
                {
                    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: OutofRange error, function=%d, value=%d\n",
                              driverName, functionName, uiFeatureId, value);
                    return asynError;
                }
                pFeatureValue->stVariant.stTriggerOption.iDelayTime = value;
                break;
            }
            else 
            {
                if ( !IsInRangeU(value, pFeatureDesc->stTriggerOption.stRangeFrameCount) )
                {
                    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: OutofRange error, function=%d, value=%d\n",
                              driverName, functionName, uiFeatureId, value);
                    return asynError;
                }
                pFeatureValue->stVariant.stTriggerOption.uiFrameCount = value;
                break;
            }
        case    edesc_Position:
            if ( subFeature == 0)
            {
                if ( !IsInRangeU(value, pFeatureDesc->stPosition.stMin.uiX,
                                 pFeatureDesc->stPosition.stMax.uiX, pFeatureDesc->stPosition.stRes.uiX) )
                {
                    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: OutofRange error, function=%d, value=%d min=%d max=%d res=%d\n",
                              driverName, functionName, uiFeatureId, value, 
                              pFeatureDesc->stPosition.stMin.uiX,
                              pFeatureDesc->stPosition.stMax.uiX,
                              pFeatureDesc->stPosition.stRes.uiX);
                    return asynError;
                }
                pFeatureValue->stVariant.stPosition.uiX = value;
                break;
            }
            else
            {
                if ( !IsInRangeU(value, pFeatureDesc->stPosition.stMin.uiY,
                                 pFeatureDesc->stPosition.stMax.uiY, pFeatureDesc->stPosition.stRes.uiY) )
                {
                    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: OutofRange error, function=%d, value=%d min=%d max=%d res=%d\n",
                              driverName, functionName, uiFeatureId, value,
                              pFeatureDesc->stPosition.stMin.uiY,
                              pFeatureDesc->stPosition.stMax.uiY,
                              pFeatureDesc->stPosition.stRes.uiY);
                    return asynError;
                }
                pFeatureValue->stVariant.stPosition.uiY = value;
                break;
            }
    }

    status = SetFeature(pFeatureValue->uiFeatureId);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Finish \n", driverName,
              functionName);
    return status;
}

/** Sets an float64 parameter.
  * \param[in] pasynUser asynUser structure that contains the function code in pasynUser->reason.
  * \param[in] value The value for this parameter
*/
asynStatus KsCam::writeFloat64( asynUser *pasynUser, epicsFloat64 value)
{
    static const char *functionName = "writeFloat64";
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
 
    if (function == ADAcquireTime)
    {
        status = SetFloatValue(eExposureTime, value*1000000.);
    }
    else if (function == ADGain)
    {
        status = SetFloatValue(eGain,value);
    }
    else if (function == ExposureTimeLimit)
    {
        status = SetFloatValue(eExposureTimeLimit, value*1000000.);
    }

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: error, status=%d function=%d, value=%d\n",
                  driverName, functionName, status, function, value);
    else{
        status = setDoubleParam(function, value);
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                  "%s:%s: function=%d, value=%d\n",
                  driverName, functionName, function, value);
    }
    callParamCallbacks();
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s::%s function=%d, value=%f, status=%d\n",
              driverName, functionName, function, value, status);
    return status;
}

asynStatus KsCam::SetFloatValue(lx_uint32 uiFeatureId, epicsFloat64 value)
{
    static const char *functionName = "SetFloatValue";
    asynStatus status = asynSuccess;
    CAM_FeatureValue*   pFeatureValue;
    CAM_FeatureDesc*    pFeatureDesc;

    uiFeatureId   = m_mapFeatureIndex[uiFeatureId];
    pFeatureValue = &m_vectFeatureValue.pstFeatureValue[uiFeatureId];
    pFeatureDesc  = &m_pFeatureDesc[uiFeatureId];

    switch(pFeatureValue->stVariant.eVarType)
    {
        case    evrt_int32:
            if ( !IsInRange(value, pFeatureDesc->stRange) )
            {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s:%s: OutofRange error, function=%d, value=%d\n",
                          driverName, functionName, uiFeatureId, value);
                return asynError;
            }
            pFeatureValue->stVariant.i32Value = value;
            break;
        case    evrt_uint32:
            if ( !IsInRangeU(value, pFeatureDesc->stRange) )
            {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s:%s: OutofRange error, function=%d, value=%d\n",
                          driverName, functionName, uiFeatureId, value);
                return asynError;
            }
            if ( pFeatureValue->uiFeatureId == eExposureTime
                    || pFeatureValue->uiFeatureId == eExposureTimeLimit )
            {
                /*Check to make sure the time is properly formatted, if not fix it*/
                if ( !CheckExposureTime(value) )
                {
                    value = AdjustExposureTime(value);
                }
            }
            pFeatureValue->stVariant.ui32Value = value;
            break;
    }
    status = SetFeature(pFeatureValue->uiFeatureId);
    return status;
}

/* Following 11 functions copied from NikonKsCamera example code */
lx_wchar* KsCam::ConvFeatureIdToName(const lx_uint32 uiFeatureId)
{
    lx_wchar*       pwszName;
    lx_uint32       i;

    for ( i=0; ; i++ )
    {
        if ( stFeatureNameRef[i].uiFeatureId == eUnknown )
        {
            break;
        }
        if ( stFeatureNameRef[i].uiFeatureId == uiFeatureId )
        {
            break;
        }
    }
    pwszName = (lx_wchar*)stFeatureNameRef[i].wszName;
    return pwszName;
}

CString KsCam::GetFeatureDispString(CAM_FeatureValue* pstFeatureValue)
{
    lx_uint32   uiIndex;
    CString     strValue;

    uiIndex = m_mapFeatureIndex[pstFeatureValue->uiFeatureId];
    CAM_FeatureDesc* pFeatureDesc = &m_pFeatureDesc[uiIndex];

    strValue = "(unknown)";
    switch( pFeatureDesc->eFeatureDescType )
    {
    case edesc_ElementList:
    {
        lx_uint32 i;
        for (i = 0; i < pFeatureDesc->uiListCount; i++)
        {
            if (pFeatureDesc->stElementList[i].varValue == pstFeatureValue->stVariant)
            {
                strValue = pFeatureDesc->stElementList[i].wszComment;
                break;
            }
        }
    }
    break;
    case edesc_Range:
    {
        switch (pstFeatureValue->stVariant.eVarType)
        {
        case evrt_int32:
            strValue.Format("%d", pstFeatureValue->stVariant.i32Value);
            break;
        case evrt_uint32:
            strValue.Format("%d", pstFeatureValue->stVariant.ui32Value);
            break;
        case evrt_MultiExposureTime:
            strValue.Format("%d", pstFeatureValue->stVariant.stMultiExposureTime.uiNum);

            if (pstFeatureValue->stVariant.stMultiExposureTime.uiNum > 0)
            {
                strValue += "(";
                lx_uint32 i;
                for (i = 0; i < pstFeatureValue->stVariant.stMultiExposureTime.uiNum; i++)
                {
                    if (i != 0)
                        strValue += ",";
                    CString strTemp;
                    strTemp.Format("%d", pstFeatureValue->stVariant.stMultiExposureTime.uiExposureTime[i]);
                    strValue += strTemp;
                }
                strValue += ")";
            }
            break;
        }
    }
    break;
    case edesc_Area:
    {
        strValue.Format("%d,%d,%d,%d",
                        pstFeatureValue->stVariant.stArea.uiLeft,
                        pstFeatureValue->stVariant.stArea.uiTop,
                        pstFeatureValue->stVariant.stArea.uiWidth,
                        pstFeatureValue->stVariant.stArea.uiHeight);
    }
    break;
    case edesc_Position:
    {
        strValue.Format("%d,%d",
                        pstFeatureValue->stVariant.stPosition.uiX,
                        pstFeatureValue->stVariant.stPosition.uiY);
    }
    break;
    case edesc_TriggerOption:
    {
        strValue.Format("%d,%d",
                        pstFeatureValue->stVariant.stTriggerOption.uiFrameCount,
                        pstFeatureValue->stVariant.stTriggerOption.iDelayTime);
    }
    break;
    case edesc_FormatList:
    {
        lx_uint32 i;
        for (i = 0; i < pFeatureDesc->uiListCount; i++)
        {
            if (pFeatureDesc->stFormatList[i].stFormat == pstFeatureValue->stVariant.stFormat)
            {
                strValue = pFeatureDesc->stFormatList[i].wszComment;
                break;
            }
        }
    }
    break;
    }

    return strValue;
}

BOOL KsCam::IsInRange(lx_int32 iTarget, CAM_FeatureDescRange& stRange)
{
    return IsInRange(iTarget, stRange.stMin.i32Value, stRange.stMax.i32Value,
                     stRange.stRes.i32Value);
}

BOOL KsCam::IsInRange(lx_int32 target, lx_int32 min, lx_int32 max, lx_int32 pitch/*=1*/)
{
    if ( target < min )
    {
        return FALSE;
    }
    if ( target > max )
    {
        return FALSE;
    }
    if ( pitch != 0 && pitch != 1 && ((target - min) % pitch) != 0 )
    {
        return FALSE;
    }
    return TRUE;
}

BOOL KsCam::IsInRangeU(lx_uint32 uiTarget, CAM_FeatureDescRange& stRange)
{
    return IsInRangeU(uiTarget, stRange.stMin.ui32Value, stRange.stMax.ui32Value,
                      stRange.stRes.ui32Value);
}

BOOL KsCam::IsInRangeU(lx_uint32 target, lx_uint32 min, lx_uint32 max,
                     lx_uint32 pitch/*=1*/)
{
    if ( target < min )     return FALSE;
    if ( target > max )     return FALSE;
    if ( pitch != 0 && pitch != 1 && ((target - min) % pitch) != 0 )    return FALSE;
    return TRUE;
}

lx_uint32 KsCam::AdjustExposureTime(lx_uint32 uiValue)
{
    lx_uint32   len, i, dev, val;
    CString     strValue;

    strValue.Format("%u", uiValue);
    len = strValue.GetLength();

    if ( len >= 6 )
    {
        dev = 1;
        for( i=0; i<len-3; i++ )
        {
            dev *= 10;
        }
        val = (uiValue / dev) * dev;
    }
    else if ( len >= 3 )
    {
        val = (uiValue / 100) * 100;
    }
    else
    {
        val = 100;
    }
    return val;
}

BOOL KsCam::CheckExposureTime(lx_uint32 uiValue)
{
    lx_int32    len;
    CString     strValue;
    strValue.Format("%d", uiValue);
    len = strValue.GetLength();
    if ( len < 3 )
    {
        return FALSE;
    }
    strValue.TrimRight(L'0');
    len = strValue.GetLength();
    if ( len > 3 )
    {
        return FALSE;
    }
    return TRUE;
}

void KsCam::FrameRateStart()
{
    m_dwStartTick = GetTickCountQuality();
    m_dwStartTickAve = 0;
    m_dwCount = 0;
    m_dwCountAve = 0;
}

/* Frame rate is calculated in driver */
void KsCam::FrameRate()
{
    double  dFrameRate, dAve, dElapsed;
    DWORD   dwNow;
    CString strText;

    dwNow = GetTickCountQuality();

    ++m_dwCount;
    if ( m_dwStartTickAve )
    {
        ++m_dwCountAve;
    }

    dElapsed = dwNow - m_dwStartTick;
    if ( dElapsed > DEF_UNIT )
    {
        dFrameRate = (double)m_dwCount * DEF_UNIT / dElapsed;
        if ( m_dwStartTickAve )
        {
            dAve = (double)m_dwCountAve * DEF_UNIT / (dwNow - m_dwStartTickAve);
        }
        else
        {
            m_dwStartTickAve = dwNow;
            m_dwCountAve = 0;
            dAve = 0.0;
        }

        if ( dFrameRate > DEF_GOSA )
        {
            setDoubleParam(NikonFrameRate, round( dFrameRate * 1000.0 ) / 1000.0);
            callParamCallbacks();
        }
        m_dwStartTick = dwNow;
        m_dwCount = 0;
    }
}

DWORD KsCam::GetTickCountQuality()
{
    DWORD           dwTick;
    LARGE_INTEGER   li_freq, li_count;

    if ( QueryPerformanceFrequency(&li_freq) )
    {
        double  unit = 1000000.0;
        QueryPerformanceCounter(&li_count);
        dwTick = (DWORD)(li_count.QuadPart * (unit / li_freq.QuadPart));
    }
    else
    {
        dwTick = GetTickCount();
        dwTick *= 1000;
    }

    return dwTick;
}


/* Code for iocsh registration */
static const iocshArg KsCamConfigArg0 = {"Port name", iocshArgString};
static const iocshArg KsCamConfigArg1 = {"Camera number", iocshArgInt};
static const iocshArg KsCamConfigArg2 = {"maxBuffers", iocshArgInt};
static const iocshArg KsCamConfigArg3 = {"maxMemory", iocshArgInt};
static const iocshArg KsCamConfigArg4 = {"priority", iocshArgInt};
static const iocshArg KsCamConfigArg5 = {"stackSize", iocshArgInt};
static const iocshArg * const KsCamConfigArgs[] = {&KsCamConfigArg0, &KsCamConfigArg1,
                                                 &KsCamConfigArg2,
                                                 &KsCamConfigArg3,
                                                 &KsCamConfigArg4,
                                                 &KsCamConfigArg5,
                                                };
static const iocshFuncDef configKsCam = {"KsCamConfig", 6, KsCamConfigArgs};
static void configKsCamCallFunc(const iocshArgBuf *args)
{
    KsCamConfig(args[0].sval, args[1].ival, args[2].ival,
              args[3].ival, args[4].ival, args[5].ival);
}

static void KsRegister(void)
{
    iocshRegister(&configKsCam, configKsCamCallFunc);
}

extern "C" {
    epicsExportRegistrar(KsRegister);
}
