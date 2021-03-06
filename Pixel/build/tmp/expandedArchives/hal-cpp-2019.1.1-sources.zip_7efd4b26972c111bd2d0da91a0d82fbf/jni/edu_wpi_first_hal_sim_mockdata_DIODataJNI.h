/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class edu_wpi_first_hal_sim_mockdata_DIODataJNI */

#ifndef _Included_edu_wpi_first_hal_sim_mockdata_DIODataJNI
#define _Included_edu_wpi_first_hal_sim_mockdata_DIODataJNI
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     edu_wpi_first_hal_sim_mockdata_DIODataJNI
 * Method:    registerInitializedCallback
 * Signature: (ILedu/wpi/first/hal/sim/NotifyCallback;Z)I
 */
JNIEXPORT jint JNICALL Java_edu_wpi_first_hal_sim_mockdata_DIODataJNI_registerInitializedCallback
  (JNIEnv *, jclass, jint, jobject, jboolean);

/*
 * Class:     edu_wpi_first_hal_sim_mockdata_DIODataJNI
 * Method:    cancelInitializedCallback
 * Signature: (II)V
 */
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_sim_mockdata_DIODataJNI_cancelInitializedCallback
  (JNIEnv *, jclass, jint, jint);

/*
 * Class:     edu_wpi_first_hal_sim_mockdata_DIODataJNI
 * Method:    getInitialized
 * Signature: (I)Z
 */
JNIEXPORT jboolean JNICALL Java_edu_wpi_first_hal_sim_mockdata_DIODataJNI_getInitialized
  (JNIEnv *, jclass, jint);

/*
 * Class:     edu_wpi_first_hal_sim_mockdata_DIODataJNI
 * Method:    setInitialized
 * Signature: (IZ)V
 */
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_sim_mockdata_DIODataJNI_setInitialized
  (JNIEnv *, jclass, jint, jboolean);

/*
 * Class:     edu_wpi_first_hal_sim_mockdata_DIODataJNI
 * Method:    registerValueCallback
 * Signature: (ILedu/wpi/first/hal/sim/NotifyCallback;Z)I
 */
JNIEXPORT jint JNICALL Java_edu_wpi_first_hal_sim_mockdata_DIODataJNI_registerValueCallback
  (JNIEnv *, jclass, jint, jobject, jboolean);

/*
 * Class:     edu_wpi_first_hal_sim_mockdata_DIODataJNI
 * Method:    cancelValueCallback
 * Signature: (II)V
 */
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_sim_mockdata_DIODataJNI_cancelValueCallback
  (JNIEnv *, jclass, jint, jint);

/*
 * Class:     edu_wpi_first_hal_sim_mockdata_DIODataJNI
 * Method:    getValue
 * Signature: (I)Z
 */
JNIEXPORT jboolean JNICALL Java_edu_wpi_first_hal_sim_mockdata_DIODataJNI_getValue
  (JNIEnv *, jclass, jint);

/*
 * Class:     edu_wpi_first_hal_sim_mockdata_DIODataJNI
 * Method:    setValue
 * Signature: (IZ)V
 */
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_sim_mockdata_DIODataJNI_setValue
  (JNIEnv *, jclass, jint, jboolean);

/*
 * Class:     edu_wpi_first_hal_sim_mockdata_DIODataJNI
 * Method:    registerPulseLengthCallback
 * Signature: (ILedu/wpi/first/hal/sim/NotifyCallback;Z)I
 */
JNIEXPORT jint JNICALL Java_edu_wpi_first_hal_sim_mockdata_DIODataJNI_registerPulseLengthCallback
  (JNIEnv *, jclass, jint, jobject, jboolean);

/*
 * Class:     edu_wpi_first_hal_sim_mockdata_DIODataJNI
 * Method:    cancelPulseLengthCallback
 * Signature: (II)V
 */
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_sim_mockdata_DIODataJNI_cancelPulseLengthCallback
  (JNIEnv *, jclass, jint, jint);

/*
 * Class:     edu_wpi_first_hal_sim_mockdata_DIODataJNI
 * Method:    getPulseLength
 * Signature: (I)D
 */
JNIEXPORT jdouble JNICALL Java_edu_wpi_first_hal_sim_mockdata_DIODataJNI_getPulseLength
  (JNIEnv *, jclass, jint);

/*
 * Class:     edu_wpi_first_hal_sim_mockdata_DIODataJNI
 * Method:    setPulseLength
 * Signature: (ID)V
 */
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_sim_mockdata_DIODataJNI_setPulseLength
  (JNIEnv *, jclass, jint, jdouble);

/*
 * Class:     edu_wpi_first_hal_sim_mockdata_DIODataJNI
 * Method:    registerIsInputCallback
 * Signature: (ILedu/wpi/first/hal/sim/NotifyCallback;Z)I
 */
JNIEXPORT jint JNICALL Java_edu_wpi_first_hal_sim_mockdata_DIODataJNI_registerIsInputCallback
  (JNIEnv *, jclass, jint, jobject, jboolean);

/*
 * Class:     edu_wpi_first_hal_sim_mockdata_DIODataJNI
 * Method:    cancelIsInputCallback
 * Signature: (II)V
 */
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_sim_mockdata_DIODataJNI_cancelIsInputCallback
  (JNIEnv *, jclass, jint, jint);

/*
 * Class:     edu_wpi_first_hal_sim_mockdata_DIODataJNI
 * Method:    getIsInput
 * Signature: (I)Z
 */
JNIEXPORT jboolean JNICALL Java_edu_wpi_first_hal_sim_mockdata_DIODataJNI_getIsInput
  (JNIEnv *, jclass, jint);

/*
 * Class:     edu_wpi_first_hal_sim_mockdata_DIODataJNI
 * Method:    setIsInput
 * Signature: (IZ)V
 */
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_sim_mockdata_DIODataJNI_setIsInput
  (JNIEnv *, jclass, jint, jboolean);

/*
 * Class:     edu_wpi_first_hal_sim_mockdata_DIODataJNI
 * Method:    registerFilterIndexCallback
 * Signature: (ILedu/wpi/first/hal/sim/NotifyCallback;Z)I
 */
JNIEXPORT jint JNICALL Java_edu_wpi_first_hal_sim_mockdata_DIODataJNI_registerFilterIndexCallback
  (JNIEnv *, jclass, jint, jobject, jboolean);

/*
 * Class:     edu_wpi_first_hal_sim_mockdata_DIODataJNI
 * Method:    cancelFilterIndexCallback
 * Signature: (II)V
 */
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_sim_mockdata_DIODataJNI_cancelFilterIndexCallback
  (JNIEnv *, jclass, jint, jint);

/*
 * Class:     edu_wpi_first_hal_sim_mockdata_DIODataJNI
 * Method:    getFilterIndex
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL Java_edu_wpi_first_hal_sim_mockdata_DIODataJNI_getFilterIndex
  (JNIEnv *, jclass, jint);

/*
 * Class:     edu_wpi_first_hal_sim_mockdata_DIODataJNI
 * Method:    setFilterIndex
 * Signature: (II)V
 */
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_sim_mockdata_DIODataJNI_setFilterIndex
  (JNIEnv *, jclass, jint, jint);

/*
 * Class:     edu_wpi_first_hal_sim_mockdata_DIODataJNI
 * Method:    resetData
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_sim_mockdata_DIODataJNI_resetData
  (JNIEnv *, jclass, jint);

#ifdef __cplusplus
}
#endif
#endif
