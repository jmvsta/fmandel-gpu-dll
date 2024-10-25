#include <jni.h>
#include "kernel.cuh"

extern "C" JNIEXPORT jlong JNICALL Java_MandelbrotLib_allocateImage(JNIEnv *env, jobject obj, jint width, jint height) {
    auto image = allocate_image(width, height);
    return (jlong) image;
}

extern "C" JNIEXPORT void JNICALL Java_MandelbrotLib_freeImage(JNIEnv *env, jobject obj, jlong imagePtr) {
    auto image = (unsigned char *) imagePtr;
    free_image(image);
}

// This function will generate the src set
extern "C" JNIEXPORT void
JNICALL Java_MandelbrotLib_generateMandelbrot(JNIEnv *env, jobject obj, jlong imagePtr, jint width, jint height,
                                              jdouble xCenter, jdouble yCenter, jdouble xMin, jdouble xMax,
                                              jdouble yMin, jdouble yMax, jint maxIter, jint zoomSteps) {
    auto *image = (unsigned char *) imagePtr;
    generate_mandelbrot(image, width, height, xCenter, yCenter, xMin, xMax, yMin, yMax, maxIter, zoomSteps);
}

extern "C" JNIEXPORT void
JNICALL Java_MandelbrotLib_saveImage(JNIEnv *env, jobject obj, jstring filename, jlong imagePtr, jint width,
                                     jint height) {
    const char *file = env->GetStringUTFChars(filename, nullptr);
    auto *image = (unsigned char *) imagePtr;
    save_image(file, image, width, height);
    env->ReleaseStringUTFChars(filename, file);
}
