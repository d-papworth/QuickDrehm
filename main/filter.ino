// Teensy Flight Controller - QuickDrehm
// Authors: Kevin Plaizer
// Version: Alpha 1.0

//========================================================================================================================//
//                                                      FILTER CODE                                                       //
//========================================================================================================================//

// Struct defines live in global_defines.h

// PT1 Low Pass filter

float pt1FilterGain(float f_cut, float dT) {
    float omega = 2.0f * M_PI * f_cut * dT;
    float cos = 1.0f - cosf(omega);
    return sqrtf(cos * (cos + 2.0f)) - cos;
}

void pt1FilterInit(pt1Filter_t *filter, float cutoff, float dT) {
    filter->state = 0.0f;
    filter->k = pt1FilterGain(cutoff, dT);
}

float pt1FilterApply(pt1Filter_t *filter, float input) {
    filter->state = filter->state + filter->k * (input - filter->state);
    return filter->state;
}

// PT2 Low Pass filter

float pt2FilterGain(float f_cut, float dT) {
    // PTn cutoff correction = 1 / sqrt(2^(1/n) - 1)
    #define CUTOFF_CORRECTION_PT2 1.553773974f

    // shift f_cut to satisfy -3dB cutoff condition
    return pt1FilterGain(f_cut * CUTOFF_CORRECTION_PT2, dT);
}

void pt2FilterInit(pt2Filter_t *filter, float cutoff, float dT) {
    filter->state = 0.0f;
    filter->state1 = 0.0f;
    filter->k = pt2FilterGain(cutoff, dT);
}

float pt2FilterApply(pt2Filter_t *filter, float input) {
    filter->state1 = filter->state1 + filter->k * (input - filter->state1);
    filter->state = filter->state + filter->k * (filter->state1 - filter->state);
    return filter->state;
}

// PT3 Low Pass filter

float pt3FilterGain(float f_cut, float dT) {
    // PTn cutoff correction = 1 / sqrt(2^(1/n) - 1)
    #define CUTOFF_CORRECTION_PT3 1.961459177f

    // shift f_cut to satisfy -3dB cutoff condition
    return pt1FilterGain(f_cut * CUTOFF_CORRECTION_PT3, dT);
}

void pt3FilterInit(pt3Filter_t *filter, float cutoff, float dT) {
    filter->state = 0.0f;
    filter->state1 = 0.0f;
    filter->state2 = 0.0f;
    filter->k = pt3FilterGain(cutoff, dT);
}

float pt3FilterApply(pt3Filter_t *filter, float input) {
    filter->state1 = filter->state1 + filter->k * (input - filter->state1);
    filter->state2 = filter->state2 + filter->k * (filter->state1 - filter->state2);
    filter->state = filter->state + filter->k * (filter->state2 - filter->state);
    return filter->state;
}

// Notch filter

void notchFilterUpdate(notchFilter_t *filter, float filterFreq, float q, float weight, float dT) {
    // setup variables
    const float omega = 2.0f * M_PI * filterFreq * dT;
    const float sn = sinf(omega);
    const float cs = cosf(omega);
    const float alpha = sn / (2.0f * q);
    const float a0_inv = 1.0 / (1.0f + alpha);

    filter->b0 = 1.0f * a0_inv;
    filter->b1 = -2.0f * cs * a0_inv;
    filter->b2 = filter->b0;
    filter->a1 = filter->b1;
    filter->a2 = (1.0f - alpha) * a0_inv;

    filter->weight = weight;
}

void notchFilterInit(notchFilter_t *filter, float filterFreq, float q, float dT) {
    notchFilterUpdate(filter, filterFreq, q, 1.0, dT);

    // zero initial samples
    filter->x1 = filter->x2 = 0.0f;
    filter->y1 = filter->y2 = 0.0f;
}

float notchFilterApply(notchFilter_t *filter, float input) {
    /* compute result */
    const float result = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 - filter->a1 * filter->y1 - filter->a2 * filter->y2;

    /* shift x1 to x2, input to x1 */
    filter->x2 = filter->x1;
    filter->x1 = input;

    /* shift y1 to y2, result to y1 */
    filter->y2 = filter->y1;
    filter->y1 = result;

    return filter->weight * result + (1.0 - filter->weight) * input;
}