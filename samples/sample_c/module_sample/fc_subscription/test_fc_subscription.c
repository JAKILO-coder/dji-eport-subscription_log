/**
 ********************************************************************
 * @file    test_fc_subscription.c
 * @brief
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <utils/util_misc.h>
#include <math.h>
#include "test_fc_subscription.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include "widget_interaction_test/test_widget_interaction.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* Private constants ---------------------------------------------------------*/
#define FC_SUBSCRIPTION_TASK_FREQ         (1)
#define FC_SUBSCRIPTION_TASK_STACK_SIZE   (1024)

/* Private types -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/
static void *UserFcSubscription_Task(void *arg);
static T_DjiReturnCode DjiTest_FcSubscriptionReceiveQuaternionCallback(const uint8_t *data, uint16_t dataSize,
                                                                       const T_DjiDataTimestamp *timestamp);

/* Private variables ---------------------------------------------------------*/
static T_DjiTaskHandle s_userFcSubscriptionThread;
static bool s_userFcSubscriptionDataShow = false;
static uint8_t s_totalSatelliteNumberUsed = 0;
static uint32_t s_userFcSubscriptionDataCnt = 0;

/* Exported functions definition ---------------------------------------------*/
T_DjiReturnCode DjiTest_FcSubscriptionStartService(void)
{
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = NULL;

    osalHandler = DjiPlatform_GetOsalHandler();
    djiStat = DjiFcSubscription_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("init data subscription module error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               DjiTest_FcSubscriptionReceiveQuaternionCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic quaternion success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic velocity error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic velocity success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic gps position error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic gps position success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic gps details error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic gps details success.");
    }

    if (osalHandler->TaskCreate("user_subscription_task", UserFcSubscription_Task,
                                FC_SUBSCRIPTION_TASK_STACK_SIZE, NULL, &s_userFcSubscriptionThread) !=
        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("user data subscription task create error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_FcSubscriptionRunSample(void)
{
    // todo list: (JAKILO on 20240523)
    // 1. try to save data to .log file
    // 2. try to load all data in document
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiFcSubscriptionQuaternion quaternion = {0};
    T_DjiFcSubscriptionAccelerationGround accelerationground = {0};
    T_DjiFcSubscriptionAccelerationBody accelerationbody = {0};
    T_DjiFcSubscriptionAccelerationRaw accelerationraw = {0};
    T_DjiFcSubscriptionVelocity velocity = {0};
    T_DjiFcSubscriptionAngularRateFusioned angularratefusioned = {0};
    T_DjiFcSubscriptionAngularRateRaw angularrateraw = {0};
    T_DjiFcSubscriptionAltitudeFused altitudefused = 0.0;
    T_DjiFcSubscriptionAltitudeBarometer altitudebarometer = 0.0;
    T_DjiFcSubscriptionAltitudeOfHomePoint altitudeofhomepoint = 0.0;
    T_DjiFcSubscriptionHeightFusion heightfusion = 0.0;
    T_DjiFcSubscriptionHeightRelative heightrelative = 0.0;
    T_DjiFcSubscriptionPositionFused positionfused = {0};
    T_DjiFcSubscriptionGpsDate gpsdate = 0;
    T_DjiFcSubscriptionGpsTime gpstime = 0;
    T_DjiFcSubscriptionGpsPosition gpsposition = {0};
    T_DjiFcSubscriptionGpsVelocity gpsvelocity = {0};
    T_DjiFcSubscriptionGpsSignalLevel gpssignallevel = 0;
    T_DjiFcSubscriptionRtkPosition rtkposition = {0};
    T_DjiFcSubscriptionRtkVelocity rtkvelocity = {0};
    T_DjiFcSubscriptionRtkYaw rtkyaw = 0;
    T_DjiFcSubscriptionRtkPositionInfo rtkpositioninfo = 0;
    T_DjiFcSubscriptionRtkYawInfo rtkyawinfo = 0;
    T_DjiFcSubscriptionCompass compass = {0};
    T_DjiFcSubscriptionRC rc = {0};
    T_DjiFcSubscriptionGimbalAngles gimbalangles = {0};
    T_DjiFcSubscriptionFlightStatus flightstatus = 0;
    T_DjiFcSubscriptionDisplaymode displaymode = 0;
    T_DjiFcSubscriptionLandinggear landinggear = 0;
    T_DjiFcSubscriptionMotorStartError motorstarterror = 0;
    T_DjiFcSubscriptionGpsControlLevel gpscontrollevel = 0;
    T_DjiFcSubscriptionRTKConnectStatus rtkconnectstatus = {0};
    T_DjiFcSubscriptionGimbalControlMode gimbalcontrolmode = 0;
    T_DjiFcSubscriptionPositionVO positionvo = {0};
    T_DjiFcSubscriptionHomePointInfo homepointinfo = {0};
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionSingleBatteryInfo singleBatteryInfo = {0};

    USER_LOG_INFO("Fc subscription sample start");
    s_userFcSubscriptionDataShow = true;

    USER_LOG_INFO("--> Step 1: Init fc subscription module");
    djiStat = DjiFcSubscription_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("init data subscription module error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--> Step 2: Subscribe all topics");
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_GROUND, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_BODY, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_RAW, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_RAW, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_BAROMETER, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_RELATIVE, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_SIGNAL_LEVEL, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW_INFO, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_COMPASS, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_STATUS, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_LANDINGGEAR, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_MOTOR_START_ERROR, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_CONTROL_LEVEL, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_CONNECT_STATUS, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_CONTROL_MODE, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO, DJI_DATA_SUBSCRIPTION_TOPIC_200_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_SET_STATUS, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_INFO, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX2, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--> Step 3: Get latest value of the subscribed topics in the next 10 seconds\r\n");

	time_t rawtime;
    struct tm *timeinfo;
    char filename[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(filename, sizeof(filename), "/home/jakilo/Documents/JAKILO_flightlog_%Y-%m-%d_%H-%M-%S.txt", timeinfo);

    FILE *fptr;
    fptr = fopen(filename, "a");
    if(fptr == NULL)
    {
        USER_LOG_ERROR("open file failed");
    }
    else
    {
        USER_LOG_INFO("open file success");
    }
    fclose(fptr);

    for (int i = 0; i < 10; ++i) {
        fptr = fopen(filename, "a");
        if(fptr == NULL)
        {
            USER_LOG_ERROR("open file failed");
        }
        else
        {
            USER_LOG_INFO("open file success");
        }
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        // printf("%Y-%m-%d_%H-%M-%S\n", timeinfo);
        fprintf(fptr, "%d-%02d-%02d_%02d-%02d-%02d\n", timeinfo->tm_year + 1900, timeinfo->tm_mon +1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

        osalHandler->TaskSleepMs(1000 / FC_SUBSCRIPTION_TASK_FREQ);
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                          (uint8_t *) &quaternion,
                                                          sizeof(T_DjiFcSubscriptionQuaternion),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic quaternion error.");
        } else {
            dji_f64_t pitch, yaw, roll;

            pitch = (dji_f64_t) asinf(-2 * quaternion.q1 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q2) * 57.3;
            roll = (dji_f64_t) atan2f(2 * quaternion.q2 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q1, -2 * quaternion.q1 * quaternion.q1 - 2 * quaternion.q2 * quaternion.q2 + 1) * 57.3;
            yaw = (dji_f64_t) atan2f(2 * quaternion.q1 * quaternion.q2 + 2 * quaternion.q0 * quaternion.q3, -2 * quaternion.q2 * quaternion.q2 - 2 * quaternion.q3 * quaternion.q3 + 1) * 57.3;

            USER_LOG_INFO("quaternion: %f %f %f %f.\n", quaternion.q0, quaternion.q1, quaternion.q2, quaternion.q3);
            fprintf(fptr, "quaternion: %f %f %f %f.\n", quaternion.q0, quaternion.q1, quaternion.q2, quaternion.q3);
            USER_LOG_INFO("euler angles: pitch = %.2f roll = %.2f yaw = %.2f.\n", pitch, yaw, roll);
            fprintf(fptr, "euler angles: pitch = %.2f roll = %.2f yaw = %.2f.\n", pitch, yaw, roll);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_GROUND,
                                                          (uint8_t *) &accelerationground,
                                                          sizeof(T_DjiFcSubscriptionAccelerationGround),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic accelerationground error.");
        } else {
            USER_LOG_INFO("acceleration ground: x:%f y:%f z:%f.\n", accelerationground.x, accelerationground.y, accelerationground.z);
            fprintf(fptr, "acceleration ground: x:%f y:%f z:%f.\n", accelerationground.x, accelerationground.y, accelerationground.z);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_BODY,
                                                          (uint8_t *) &accelerationbody,
                                                          sizeof(T_DjiFcSubscriptionAccelerationBody),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic accelerationbody error.");
        } else {
            USER_LOG_INFO("acceleration body: x:%f y:%f z:%f.\n", accelerationbody.x, accelerationbody.y, accelerationbody.z);
            fprintf(fptr, "acceleration body: x:%f y:%f z:%f.\n", accelerationbody.x, accelerationbody.y, accelerationbody.z);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_RAW,
                                                          (uint8_t *) &accelerationraw,
                                                          sizeof(T_DjiFcSubscriptionAccelerationRaw),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic accelerationraw error.");
        } else {
            USER_LOG_INFO("acceleration raw: x:%f y:%f z:%f.\n", accelerationraw.x, accelerationraw.y, accelerationraw.z);
            fprintf(fptr, "acceleration raw: x:%f y:%f z:%f.\n", accelerationraw.x, accelerationraw.y, accelerationraw.z);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                                          (uint8_t *) &velocity,
                                                          sizeof(T_DjiFcSubscriptionVelocity),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic velocity error.");
        } else {
            USER_LOG_INFO("velocity: x = %f y = %f z = %f healthFlag = %d, timestamp ms = %d us = %d.\n", velocity.data.x,
                          velocity.data.y,
                          velocity.data.z, velocity.health, timestamp.millisecond, timestamp.microsecond);
            fprintf(fptr, "velocity: x = %f y = %f z = %f healthFlag = %d, timestamp ms = %d us = %d.\n", velocity.data.x,
                          velocity.data.y,
                          velocity.data.z, velocity.health, timestamp.millisecond, timestamp.microsecond);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED,
                                                          (uint8_t *) &angularratefusioned,
                                                          sizeof(T_DjiFcSubscriptionAngularRateFusioned),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic angular rate fusioned error.");
        } else {
            USER_LOG_INFO("angular rate fusioned x = %f, y = %f, z = %f\n", angularratefusioned.x, angularratefusioned.y, angularratefusioned.z);
            fprintf(fptr, "angular rate fusioned x = %f, y = %f, z = %f\n", angularratefusioned.x, angularratefusioned.y, angularratefusioned.z);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_RAW,
                                                          (uint8_t *) &angularrateraw,
                                                          sizeof(T_DjiFcSubscriptionAngularRateRaw),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic angular rate raw error.");
        } else {
            USER_LOG_INFO("angular rate raw x = %f, y = %f, z = %f\n", angularrateraw.x, angularrateraw.y, angularrateraw.z);
            fprintf(fptr, "angular rate raw x = %f, y = %f, z = %f\n", angularrateraw.x, angularrateraw.y, angularrateraw.z);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
                                                          (uint8_t *) &altitudefused,
                                                          sizeof(T_DjiFcSubscriptionAltitudeFused),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic altitude fused error.");
        } else {
            USER_LOG_INFO("altitude fused = %f\n", altitudefused);
            fprintf(fptr, "altitude fused = %f\n", altitudefused);
        }


        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_BAROMETER,
                                                          (uint8_t *) &altitudebarometer,
                                                          sizeof(T_DjiFcSubscriptionAltitudeBarometer),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic altitude barometer error.");
        } else {
            USER_LOG_INFO("altitude barometer = %f\n", altitudebarometer);
            fprintf(fptr, "altitude barometer = %f\n", altitudebarometer);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
                                                          (uint8_t *) &altitudeofhomepoint,
                                                          sizeof(T_DjiFcSubscriptionAltitudeOfHomePoint),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic altitude of home point error.");
        } else {
            USER_LOG_INFO("altitude of home point = %f\n", altitudeofhomepoint);
            fprintf(fptr, "altitude of home point = %f\n", altitudeofhomepoint);
        }
 
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
                                                          (uint8_t *) &heightfusion,
                                                          sizeof(T_DjiFcSubscriptionHeightFusion),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic height of fusion error.");
        } else {
            USER_LOG_INFO("height of fusion = %f\n", heightfusion);
            fprintf(fptr, "height of fusion = %f\n", heightfusion);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_RELATIVE,
                                                          (uint8_t *) &heightrelative,
                                                          sizeof(T_DjiFcSubscriptionHeightRelative),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic height relative error.");
        } else {
            USER_LOG_INFO("height relative = %f\n", heightrelative);
            fprintf(fptr, "height relative = %f\n", heightrelative);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
                                                          (uint8_t *) &positionfused,
                                                          sizeof(T_DjiFcSubscriptionPositionFused),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic position fused error.");
        } else {
            USER_LOG_INFO("positionfused longitude = %f, latitude = %f, altitude = %f visibleSatelliteNumber = %d\n", positionfused.longitude, positionfused.latitude, positionfused.altitude, positionfused.visibleSatelliteNumber);
            fprintf(fptr, "positionfused longitude = %f, latitude = %f, altitude = %f visibleSatelliteNumber = %d\n", positionfused.longitude, positionfused.latitude, positionfused.altitude, positionfused.visibleSatelliteNumber);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE,
                                                          (uint8_t *) &gpsdate,
                                                          sizeof(T_DjiFcSubscriptionGpsDate),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps date error.");
        } else {
            USER_LOG_INFO("gps date = %d\n", gpsdate);
            fprintf(fptr, "gps date = %d\n", gpsdate);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME,
                                                          (uint8_t *) &gpstime,
                                                          sizeof(T_DjiFcSubscriptionGpsTime),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps time error.");
        } else {
            USER_LOG_INFO("gps time = %d\n", gpstime);
            fprintf(fptr, "gps time = %d\n", gpstime);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                          (uint8_t *) &gpsposition,
                                                          sizeof(T_DjiFcSubscriptionGpsPosition),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps position error.");
        } else {
            USER_LOG_INFO("gps position: x = %d y = %d z = %d.\n", gpsposition.x, gpsposition.y, gpsposition.z);
            fprintf(fptr, "gps position: x = %d y = %d z = %d.\n", gpsposition.x, gpsposition.y, gpsposition.z);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_VELOCITY,
                                                          (uint8_t *) &gpsvelocity,
                                                          sizeof(T_DjiFcSubscriptionGpsVelocity),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps velocity error.");
        } else {
            USER_LOG_INFO("gps velocity: x = %f y = %f z = %f.\n", gpsvelocity.x, gpsvelocity.y, gpsvelocity.z);
            fprintf(fptr, "gps velocity: x = %f y = %f z = %f.\n", gpsvelocity.x, gpsvelocity.y, gpsvelocity.z);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_SIGNAL_LEVEL,
                                                          (uint8_t *) &gpssignallevel,
                                                          sizeof(T_DjiFcSubscriptionGpsSignalLevel),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps signal level error.");
        } else {
            USER_LOG_INFO("gps signal level = %d.\n", gpssignallevel);
            fprintf(fptr, "gps signal level = %d.\n", gpssignallevel);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
                                                          (uint8_t *) &rtkposition,
                                                          sizeof(T_DjiFcSubscriptionRtkPosition),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic rtk position error.");
        } else {
            USER_LOG_INFO("rtk position: longitude = %f, latitude = %f, hfsl = %f.\n", rtkposition.longitude, rtkposition.latitude, rtkposition.hfsl);
            fprintf(fptr, "rtk position: longitude = %f, latitude = %f, hfsl = %f.\n", rtkposition.longitude, rtkposition.latitude, rtkposition.hfsl);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY,
                                                          (uint8_t *) &rtkvelocity,
                                                          sizeof(T_DjiFcSubscriptionRtkVelocity),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic rtk velocity error.");
        } else {
            USER_LOG_INFO("rtk velocity : x = %f y = %f z = %f.\n", rtkvelocity.x, rtkvelocity.y, rtkvelocity.z);
            fprintf(fptr, "rtk velocity : x = %f y = %f z = %f.\n", rtkvelocity.x, rtkvelocity.y, rtkvelocity.z);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW,
                                                          (uint8_t *) &rtkyaw,
                                                          sizeof(T_DjiFcSubscriptionRtkYaw),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic rtk velocity error.");
        } else {
            USER_LOG_INFO("rtk yaw = %d.\n", rtkyaw);
            fprintf(fptr, "rtk yaw = %d.\n", rtkyaw);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
                                                          (uint8_t *) &rtkpositioninfo,
                                                          sizeof(T_DjiFcSubscriptionRtkPositionInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic rtk position info error.");
        } else {
            USER_LOG_INFO("rtk position info = %d.\n", rtkpositioninfo);
            fprintf(fptr, "rtk position info = %d.\n", rtkpositioninfo);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW_INFO,
                                                          (uint8_t *) &rtkyawinfo,
                                                          sizeof(T_DjiFcSubscriptionRtkYawInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic rtk yaw info.");
        } else {
            USER_LOG_INFO("rtk yaw info = %d.\n", rtkyawinfo);
            fprintf(fptr, "rtk yaw info = %d.\n", rtkyawinfo);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_COMPASS,
                                                          (uint8_t *) &compass,
                                                          sizeof(T_DjiFcSubscriptionCompass),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic compass.");
        } else {
            USER_LOG_INFO("compass: x = %d, y = %d, z = %d.\n", compass.x, compass.y, compass.z);
            fprintf(fptr, "compass: x = %d, y = %d, z = %d.\n", compass.x, compass.y, compass.z);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
                                                          (uint8_t *) &gimbalangles,
                                                          sizeof(T_DjiFcSubscriptionGimbalAngles),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gimbal angles.");
        } else {
            USER_LOG_INFO("gimbal angles: x = %f, y = %f, z = %f.\n", gimbalangles.x, gimbalangles.y, gimbalangles.z);
            fprintf(fptr, "gimbal angles: x = %f, y = %f, z = %f.\n", gimbalangles.x, gimbalangles.y, gimbalangles.z);
        }        

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                          (uint8_t *) &flightstatus,
                                                          sizeof(T_DjiFcSubscriptionFlightStatus),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic flight status.");
        } else {
            USER_LOG_INFO("flight status = %d.\n", flightstatus);
            fprintf(fptr, "flight status = %d.\n", flightstatus);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                                                          (uint8_t *) &displaymode,
                                                          sizeof(T_DjiFcSubscriptionDisplaymode),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic dispaly mode.");
        } else {
            USER_LOG_INFO("dispaly mode = %d.\n", displaymode);
            fprintf(fptr, "dispaly mode = %d.\n", displaymode);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_LANDINGGEAR,
                                                          (uint8_t *) &landinggear,
                                                          sizeof(T_DjiFcSubscriptionLandinggear),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic landing gear.");
        } else {
            USER_LOG_INFO("landing gear = %d.\n", landinggear);
            fprintf(fptr, "landing gear = %d.\n", landinggear);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_MOTOR_START_ERROR,
                                                          (uint8_t *) &motorstarterror,
                                                          sizeof(T_DjiFcSubscriptionMotorStartError),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic motor start error.");
        } else {
            USER_LOG_INFO("motor start error = %d.\n", motorstarterror);
            fprintf(fptr, "motor start error = %d.\n", motorstarterror);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_CONTROL_LEVEL,
                                                          (uint8_t *) &gpscontrollevel,
                                                          sizeof(T_DjiFcSubscriptionGpsControlLevel),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps control level.");
        } else {
            USER_LOG_INFO("gps control level = %d.\n", gpscontrollevel);
            fprintf(fptr, "gps control level = %d.\n", gpscontrollevel);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_CONTROL_MODE,
                                                          (uint8_t *) &gimbalcontrolmode,
                                                          sizeof(T_DjiFcSubscriptionGimbalControlMode),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gimbal control mode.");
        } else {
            USER_LOG_INFO("gimbal control mode = %d.\n", gimbalcontrolmode);
            fprintf(fptr, "gimbal control mode = %d.\n", gimbalcontrolmode);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO,
                                                          (uint8_t *) &positionvo,
                                                          sizeof(T_DjiFcSubscriptionPositionVO),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic position VO.");
        } else {
            USER_LOG_INFO("position VO: x = %f, y = %f, z = %f, xhealth = %d, yhealth = %d, zhealth = %d, reserved = %d.\n", positionvo.x, positionvo.y, positionvo.z, positionvo.xHealth, positionvo.yHealth, positionvo.zHealth, positionvo.reserved);
            fprintf(fptr, "position VO: x = %f, y = %f, z = %f, xhealth = %d, yhealth = %d, zhealth = %d, reserved = %d.\n", positionvo.x, positionvo.y, positionvo.z, positionvo.xHealth, positionvo.yHealth, positionvo.zHealth, positionvo.reserved);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_INFO,
                                                          (uint8_t *) &homepointinfo,
                                                          sizeof(T_DjiFcSubscriptionHomePointInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic home point info.");
        } else {
            USER_LOG_INFO("home point info: latitude = %f, longtitude = %f\n", homepointinfo.latitude, homepointinfo.longitude);
            fprintf(fptr, "home point info: latitude = %f, longtitude = %f\n", homepointinfo.latitude, homepointinfo.longitude);
        }


        //Attention: if you want to subscribe the single battery info on M300 RTK, you need connect USB cable to
        //OSDK device or use topic DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO instead.
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1,
                                                          (uint8_t *) &singleBatteryInfo,
                                                          sizeof(T_DjiFcSubscriptionSingleBatteryInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic battery single info index1 error.");
        } else {
            USER_LOG_INFO(
                "battery single info index1: capacity percent = %ld% voltage = %ldV temperature = %.2f degree.\n",
                singleBatteryInfo.batteryCapacityPercent,
                singleBatteryInfo.currentVoltage / 1000,
                (dji_f32_t) singleBatteryInfo.batteryTemperature / 10);
            fprintf(fptr, "battery single info index1: capacity percent = %ld% voltage = %ldV temperature = %.2f degree.\n",
                singleBatteryInfo.batteryCapacityPercent,
                singleBatteryInfo.currentVoltage / 1000,
                (dji_f32_t) singleBatteryInfo.batteryTemperature / 10);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX2,
                                                          (uint8_t *) &singleBatteryInfo,
                                                          sizeof(T_DjiFcSubscriptionSingleBatteryInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic battery single info index2 error.");
        } else {
            USER_LOG_INFO(
                "battery single info index2: capacity percent = %ld% voltage = %ldV temperature = %.2f degree.\r\n",
                singleBatteryInfo.batteryCapacityPercent,
                singleBatteryInfo.currentVoltage / 1000,
                (dji_f32_t) singleBatteryInfo.batteryTemperature / 10);
            fprintf(fptr, "battery single info index2: capacity percent = %ld% voltage = %ldV temperature = %.2f degree.\n",
                singleBatteryInfo.batteryCapacityPercent,
                singleBatteryInfo.currentVoltage / 1000,
                (dji_f32_t) singleBatteryInfo.batteryTemperature / 10);
        }
        fclose(fptr);
    }
    //fclose(fptr);
    USER_LOG_INFO("--> Step 4: Unsubscribe the topics of quaternion, velocity and gps position");
    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_GROUND);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_BODY);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_RAW);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_RAW);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_BAROMETER);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_RELATIVE);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_VELOCITY);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_SIGNAL_LEVEL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW_INFO);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_COMPASS);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_STATUS);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_LANDINGGEAR);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_MOTOR_START_ERROR);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_CONTROL_LEVEL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_CONNECT_STATUS);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_CONTROL_MODE);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_SET_STATUS);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_INFO);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--> Step 5: Deinit fc subscription module");

    djiStat = DjiFcSubscription_DeInit();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit fc subscription error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    s_userFcSubscriptionDataShow = false;
    USER_LOG_INFO("Fc subscription sample end");

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_FcSubscriptionDataShowTrigger(void)
{
    s_userFcSubscriptionDataShow = !s_userFcSubscriptionDataShow;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_FcSubscriptionGetTotalSatelliteNumber(uint8_t *number)
{
    *number = s_totalSatelliteNumberUsed;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* Private functions definition-----------------------------------------------*/
#ifndef __CC_ARM
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-noreturn"
#pragma GCC diagnostic ignored "-Wreturn-type"
#endif

static void *UserFcSubscription_Task(void *arg)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionVelocity velocity = {0};
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionGpsPosition gpsPosition = {0};
    T_DjiFcSubscriptionGpsDetails gpsDetails = {0};
    T_DjiOsalHandler *osalHandler = NULL;

    USER_UTIL_UNUSED(arg);
    osalHandler = DjiPlatform_GetOsalHandler();

    while (1) {
        osalHandler->TaskSleepMs(1000 / FC_SUBSCRIPTION_TASK_FREQ);

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                                          (uint8_t *) &velocity,
                                                          sizeof(T_DjiFcSubscriptionVelocity),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic velocity error.");
        }

        if (s_userFcSubscriptionDataShow == true) {
            USER_LOG_INFO("velocity: x %f y %f z %f, healthFlag %d.", velocity.data.x, velocity.data.y,
                          velocity.data.z, velocity.health);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                          (uint8_t *) &gpsPosition,
                                                          sizeof(T_DjiFcSubscriptionGpsPosition),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps position error.");
        }

        if (s_userFcSubscriptionDataShow == true) {
            USER_LOG_INFO("gps position: x %d y %d z %d.", gpsPosition.x, gpsPosition.y, gpsPosition.z);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
                                                          (uint8_t *) &gpsDetails,
                                                          sizeof(T_DjiFcSubscriptionGpsDetails),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps details error.");
        }

        if (s_userFcSubscriptionDataShow == true) {
            USER_LOG_INFO("gps total satellite number used: %d %d %d.",
                          gpsDetails.gpsSatelliteNumberUsed,
                          gpsDetails.glonassSatelliteNumberUsed,
                          gpsDetails.totalSatelliteNumberUsed);
            s_totalSatelliteNumberUsed = gpsDetails.totalSatelliteNumberUsed;
        }

    }
}

#ifndef __CC_ARM
#pragma GCC diagnostic pop
#endif

static T_DjiReturnCode DjiTest_FcSubscriptionReceiveQuaternionCallback(const uint8_t *data, uint16_t dataSize,
                                                                       const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionQuaternion *quaternion = (T_DjiFcSubscriptionQuaternion *) data;
    dji_f64_t pitch, yaw, roll;

    USER_UTIL_UNUSED(dataSize);

    pitch = (dji_f64_t) asinf(-2 * quaternion->q1 * quaternion->q3 + 2 * quaternion->q0 * quaternion->q2) * 57.3;
    roll = (dji_f64_t) atan2f(2 * quaternion->q2 * quaternion->q3 + 2 * quaternion->q0 * quaternion->q1,
                             -2 * quaternion->q1 * quaternion->q1 - 2 * quaternion->q2 * quaternion->q2 + 1) * 57.3;
    yaw = (dji_f64_t) atan2f(2 * quaternion->q1 * quaternion->q2 + 2 * quaternion->q0 * quaternion->q3,
                             -2 * quaternion->q2 * quaternion->q2 - 2 * quaternion->q3 * quaternion->q3 + 1) *
          57.3;

    if (s_userFcSubscriptionDataShow == true) {
        if (s_userFcSubscriptionDataCnt++ % DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ == 0) {
            USER_LOG_INFO("receive quaternion data.");
            USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp->millisecond,
                          timestamp->microsecond);
            USER_LOG_INFO("quaternion: %f %f %f %f.", quaternion->q0, quaternion->q1, quaternion->q2,
                          quaternion->q3);

            USER_LOG_INFO("euler angles: pitch = %.2f roll = %.2f yaw = %.2f.\r\n", pitch, roll, yaw);
            DjiTest_WidgetLogAppend("pitch = %.2f roll = %.2f yaw = %.2f.", pitch, roll, yaw);
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
