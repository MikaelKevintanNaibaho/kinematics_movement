#include "ik.h"

float degrees(float rad) {
    return rad * (180.0 / M_PI);
}

float radians(float deg) {
    return deg * (M_PI / 180.0);
}

float normalize_angle(float angle) {
    angle = fmodf(angle, 360.0);  // Ensure angle is within the range of -360.0 to 360.0
    
    // Convert negative angles to their corresponding positive angles within the same position
    if (angle < 0) angle += 360.0;

    // Ensure angle is within the range of 0.0 to 180.0
    if (angle > 180.0) angle = 360.0 - angle;

    return angle;
}

float *get_target(SpiderLeg *leg) {
    return leg->joints[3];
}


void set_angles(SpiderLeg *leg, float angles[3]) {
    leg->theta1 = normalize_angle(angles[0]);
    leg->theta2 = normalize_angle(angles[1]);
    leg->theta3 = normalize_angle(angles[2]);

    set_pwm_angle(SERVO_CHANNEL_1, (int)leg->theta1, PWM_FREQ);
    set_pwm_angle(SERVO_CHANNEL_2, (int)leg->theta2,PWM_FREQ );
    set_pwm_angle(SERVO_CHANNEL_3, (int)leg->theta3, PWM_FREQ);

    printf("Theta1: %.4f degrees\n", leg->theta1);
    printf("Theta2: %.4f degrees\n", leg->theta2);
    printf("Theta3: %.4f degrees\n", leg->theta3);
}

void inverse_kinematics(SpiderLeg *leg, float target[3])
{
    float x = target[0] + 100.0;
    float y = target[1] + 100.0;
    float z = target[2];
    float z_offset = 100.0 + z;
    float theta1 = atan2f(x, y);
    float L1 = sqrtf(powf(x, 2) + powf(y, 2));

    float L = sqrtf(powf(z_offset, 2) + powf(L1 - COXA_LENGTH, 2));

    float alpha1 = acosf(z_offset / L);
    
    float alpha2_cos = (powf(TIBIA_LENGTH, 2) - powf(FEMUR_LENGTH, 2) - powf(L, 2)) / (-2 * FEMUR_LENGTH * L);
    float alpha2 = acosf(alpha2_cos);

    float theta2 = alpha1 + alpha2;

    float alpha3_cos = ((powf(L, 2) - powf(TIBIA_LENGTH, 2) - powf(FEMUR_LENGTH, 2))) / (-2 * TIBIA_LENGTH * FEMUR_LENGTH);
    float alpha3 = acosf(alpha3_cos);

    float theta3 = M_PI - alpha3;
    if (theta3 >= radians(145)){
         theta3 = radians(145);
         printf("melewati batas fisik tibia kaki\n");
    }
    float angles[3] = {degrees(theta1), degrees(theta2), degrees(theta3)};
    set_angles(leg, angles);


}

void move(SpiderLeg *leg, float target[3]) {

    // Calculate the target position relative to the leg's current position
    float new_position[3];
    new_position[0] =  target[0] + leg->joints[3][0];  // Update x-coordinate relative to current position
    new_position[1] = target[1] + leg->joints[3][1];   // Update y-coordinate relative to current position
    new_position[2] = target[2] + leg->joints[3][2];   // Update z-coordinate relative to current position
    
    // Update the leg with inverse kinematics to reach the new position
    inverse_kinematics(leg, new_position);
}