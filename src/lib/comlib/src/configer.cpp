#ifndef __CONFIGER_H__
#include "configer.h"
#endif

#include <stdlib.h>
#include "tinyxml.h"

#define         STR_NODE_DEST    						"DEST"
#define         STR_ATTR_TYPE    						"TYPE"

#define         STR_NODE_CONTROL_MODIFY  				"CONTROL_MODIFY"
#define         STR_NODE_DEAD_RECKON_COMPOENSATE  		"DEAD_RECKON_COMPOENSATE"
#define     	STR_ATTR_LEFT_WHEEL_FORWARD				"left_wheel_forward"
#define     	STR_ATTR_LEFT_WHEEL_BACKWARD			"left_wheel_backward"
#define     	STR_ATTR_RIGHT_WHEEL_FORWARD			"right_wheel_forward"
#define     	STR_ATTR_RIGHT_WHEEL_BACKWARD			"right_wheel_backward"

#define         STR_NODE_CODES_TO_DISTANCE  			"CODES_TO_DISTANCE"
#define     	STR_ATTR_COFFICIENT_MODIFY				"cofficient_modify"
#define     	STR_ATTR_DRIVER_SERVO_INTERVAL			"driver_servo_interval"

#define         STR_NODE_WHEELS_SPACES_MODIFY  			"WHEELS_SPACES_MODIFY"
#define     	STR_ATTR_WHEELS_SPACES_MODIFY			"wheels_spaces_modify"

#define         STR_NODE_FAN                			"FAN"
#define     	STR_ATTR_FAN_TIME           			"fan_time"

#define         STR_NODE_SERIAL                			"SERIAL"
#define     	STR_ATTR_SERIAL_NUM           			"serial_num"

#define         STR_NODE_CHARGE_ID                		"CHARGE_ID"
#define     	STR_ATTR_CHARGE_ID           			"charge_id"

#define         STR_NODE_CHARGE_TYPE              		"CHARGE_TYPE"
#define     	STR_ATTR_CHARGE_TPYE          			"charge_tpye"
#define     	STR_ATTR_CHARGE_TYPE          			"charge_type"

#define         STR_NODE_CHARGE_VER              		"CHARGE_VER"
#define     	STR_ATTR_CHARGE_VER          			"charge_ver"

#define         STR_NODE_CHARGE_CHANNEL             		"CHARGE_CHANNEL"
#define     	STR_ATTR_CHARGE_CHANNEL         			"charge_channel"

#define         STR_NODE_INFRARED             			"INFRARED"
#define     	STR_ATTR_INFRARED_1           			"infrared_1"
#define     	STR_ATTR_INFRARED_2           			"infrared_2"
#define     	STR_ATTR_INFRARED_3           			"infrared_3"
#define     	STR_ATTR_INFRARED_4           			"infrared_4"
#define     	STR_ATTR_INFRARED_5           			"infrared_5"
#define     	STR_ATTR_INFRARED_6           			"infrared_6"
#define     	STR_ATTR_INFRARED_7           			"infrared_7"

#define         STR_NODE_SYS_VERSION           			"SYSTEM_VERSION"
#define     	STR_ATTR_SYS_VERSION           			"system_version"

#define         STR_NODE_DRIVER_VERSION        			"DRIVER_VERSION"
#define     	STR_ATTR_DRIVER_VERSION        			"driver_version"

#define         STR_NODE_ULTRA_VERSION         			"ULTRA_VERSION"
#define     	STR_ATTR_ULTRA_VERSION         			"ultra_version"

#define         STR_NODE_CHARGE_VERSION         		"CHARGE_VERSION"
#define     	STR_ATTR_CHARGE_VERSION         		"charge_version"

#define 		STR_NODE_CAMERAOLD   					"CAMERAOLD"
#define 		STR_NODE_CONFIGOLD   					"CONFIGOLD"
#define 		STR_NODE_CORESERIALOLD   				"CORESERIALOLD"
#define 		STR_NODE_INFRAREDOLD   					"INFRAREDOLD"
#define 		STR_NODE_IODETECTIONOLD   				"IODETECTIONOLD"
#define 		STR_NODE_LOGOLD   						"LOGOLD"
#define 		STR_NODE_MOTIONOLD   					"MOTIONOLD"
#define 		STR_NODE_POWERDETECTIONOLD   			"POWERDETECTIONOLD"
#define 		STR_NODE_PROTECTOLD   					"PROTECTOLD"
#define 		STR_NODE_RECALIBRATIONOLD   			"RECALIBRATIONOLD"
#define 		STR_NODE_ULTRASONICOLD   				"ULTRASONICOLD"
#define 		STR_NODE_UPGRADEOLD   					"UPGRADEOLD"
#define 		STR_ATTR_LOG_INDEX   					"index"
#define 		STR_ATTR_LOG_NAME   					"name"
#define 		STR_ATTR_LOG_SITE   					"site"
#define 		STR_ATTR_LOG_LEVEL   					"level"
#define 		STR_ATTR_LOG_MODIFY   					"modify"

#define 		STR_SERL_PAD_NAME   					"pad"
#define 		STR_SERL_MOTOR_NAME   					"ultra"
#define 		STR_SERL_ULTRA_NAME   					"motor"
#define 		STR_SERL_PORT_READ   					"read"
#define 		STR_SERL_PORT_WRITE   					"write"


CConfiger* CConfiger::m_Instance = NULL;

CConfiger::CConfiger()
{
}

CConfiger::~CConfiger()
{
    if(NULL != m_Instance)
    {
        delete m_Instance;
        m_Instance = NULL;
    }
}

CConfiger* CConfiger::GetInstance()
{
    if (NULL == m_Instance)
    {
        m_Instance = new CConfiger();
    }
    return m_Instance;

}

bool CConfiger::ReadConfig(const char* strFileName, CONFIG_DATA* pConfigData)
{
    if(NULL == strFileName || NULL == pConfigData)
        return false;

    if(!ReadDriverConfig(strFileName, &pConfigData->driver_config))
        return false;

    if(!ReadWorkConfig(strFileName, &pConfigData->work_config))
        return false;

    return true;
}

bool CConfiger::SetConfig(const char* strFileName, CONFIG_DATA* pConfigData)
{
    if(NULL == strFileName || NULL == pConfigData)
        return false;

    if(!SetDriverConfig(strFileName, &pConfigData->driver_config))
        return false;

    if(!SetWorkConfig(strFileName, &pConfigData->work_config))
        return false;

    return true;
}

bool CConfiger::CreateDriverConfig(const char* strFileName)
{
    if(NULL == strFileName)
        return false;

    TiXmlDocument* pCfgDoc = new TiXmlDocument();

    TiXmlDeclaration* decl = new TiXmlDeclaration( "1.0", "utf-8", "" );
    pCfgDoc->LinkEndChild(decl);

    TiXmlElement* pEle = NULL;
    TiXmlElement* pRoot = new TiXmlElement("Root");
    pCfgDoc->LinkEndChild(pRoot);

    //driver
    TiXmlElement* pDest = new TiXmlElement("DEST");
    pDest->SetAttribute("TYPE", "driver");
    pRoot->LinkEndChild(pDest);

    pEle = new TiXmlElement("CONTROL_MODIFY");
    pEle->SetAttribute("left_wheel_forward", "");
    pEle->SetAttribute("left_wheel_backward", "");
    pEle->SetAttribute("right_wheel_forward", "");
    pEle->SetAttribute("right_wheel_backward", "");
    pDest->LinkEndChild(pEle);

    pEle = new TiXmlElement("CODES_TO_DISTANCE");
    pEle->SetAttribute("cofficient_modify", "");
    pEle->SetAttribute("driver_servo_interval", "");
    pDest->LinkEndChild(pEle);

    pEle = new TiXmlElement("DEAD_RECKON_COMPOENSATE");
    pEle->SetAttribute("left_wheel_forward", "");
    pEle->SetAttribute("left_wheel_backward", "");
    pEle->SetAttribute("right_wheel_forward", "");
    pEle->SetAttribute("right_wheel_backward", "");
    pDest->LinkEndChild(pEle);

    pEle = new TiXmlElement("WHEELS_SPACES_MODIFY");
    pEle->SetAttribute("wheels_spaces_modify", "");
    pDest->LinkEndChild(pEle);

    pEle = new TiXmlElement("RESERVED");
    pEle->SetAttribute("reserved_1", "");
    pEle->SetAttribute("reserved_2", "");
    pEle->SetAttribute("reserved_3", "");
    pEle->SetAttribute("reserved_4", "");
    pEle->SetAttribute("reserved_5", "");
    pEle->SetAttribute("reserved_6", "");
    pEle->SetAttribute("reserved_7", "");
    pEle->SetAttribute("reserved_8", "");
    pEle->SetAttribute("reserved_9", "");
    pEle->SetAttribute("reserved_10", "");
    pDest->LinkEndChild(pEle);

    pCfgDoc->SaveFile(strFileName);

    if(NULL != pCfgDoc)
    {
        delete pCfgDoc;
        pCfgDoc = NULL;
    }

    return true;
}

bool CConfiger::CreateLocalConfig(const char* strFileName)
{
    if(NULL == strFileName)
        return false;

    TiXmlDocument* pCfgDoc = new TiXmlDocument();

    TiXmlDeclaration* decl = new TiXmlDeclaration( "1.0", "utf-8", "" );
    pCfgDoc->LinkEndChild(decl);

    TiXmlElement* pEle = NULL;
    TiXmlElement* pRoot = new TiXmlElement("Root");
    pCfgDoc->LinkEndChild(pRoot);

    //local
    TiXmlElement* pDestLocal = new TiXmlElement("DEST");
    pDestLocal->SetAttribute("TYPE", "local");
    pRoot->LinkEndChild(pDestLocal);

    pEle = new TiXmlElement("FAN");
    pEle->SetAttribute("fan_time", "");
    pDestLocal->LinkEndChild(pEle);

    pEle = new TiXmlElement("SERIAL");
    pEle->SetAttribute("serial_num", "");
    pDestLocal->LinkEndChild(pEle);

    pEle = new TiXmlElement("INFRARED");
    pEle->SetAttribute("infrared_1", "");
    pEle->SetAttribute("infrared_2", "");
    pEle->SetAttribute("infrared_3", "");
    pEle->SetAttribute("infrared_4", "");
    pEle->SetAttribute("infrared_5", "");
    pEle->SetAttribute("infrared_6", "");
    pEle->SetAttribute("infrared_7", "");
    pDestLocal->LinkEndChild(pEle);

    pEle = new TiXmlElement("RESERVED");
    pEle->SetAttribute("reserved_1", "");
    pEle->SetAttribute("reserved_2", "");
    pEle->SetAttribute("reserved_3", "");
    pEle->SetAttribute("reserved_4", "");
    pEle->SetAttribute("reserved_5", "");
    pEle->SetAttribute("reserved_6", "");
    pEle->SetAttribute("reserved_7", "");
    pEle->SetAttribute("reserved_8", "");
    pEle->SetAttribute("reserved_9", "");
    pEle->SetAttribute("reserved_10", "");
    pDestLocal->LinkEndChild(pEle);

    pCfgDoc->SaveFile(strFileName);

    if(NULL != pCfgDoc)
    {
        delete pCfgDoc;
        pCfgDoc = NULL;
    }

    return true;
}

bool CConfiger::CreateChargeConfig(const char* strFileName)
{
    if(NULL == strFileName)
        return false;

    TiXmlDocument* pCfgDoc = new TiXmlDocument();

    TiXmlDeclaration* decl = new TiXmlDeclaration( "1.0", "utf-8", "" );
    pCfgDoc->LinkEndChild(decl);

    TiXmlElement* pEle = NULL;
    TiXmlElement* pRoot = new TiXmlElement("Root");
    pCfgDoc->LinkEndChild(pRoot);

    //charge
    TiXmlElement* pDestCharge = new TiXmlElement("DEST");
    pDestCharge->SetAttribute("TYPE", "charge");
    pRoot->LinkEndChild(pDestCharge);

    pEle = new TiXmlElement("CHARGE_ID");
    pEle->SetAttribute("charge_id", "10000");
    pDestCharge->LinkEndChild(pEle);

    pEle = new TiXmlElement("CHARGE_TYPE");
    pEle->SetAttribute("charge_type", "");
    pDestCharge->LinkEndChild(pEle);

    pEle = new TiXmlElement("CHARGE_VER");
    pEle->SetAttribute("charge_ver", "");
    pDestCharge->LinkEndChild(pEle);

    pEle = new TiXmlElement("CHARGE_CHANNEL");
    pEle->SetAttribute("charge_channel", "");
    pDestCharge->LinkEndChild(pEle);

    pEle = new TiXmlElement("RESERVED");
    pEle->SetAttribute("reserved_1", "");
    pEle->SetAttribute("reserved_2", "");
    pEle->SetAttribute("reserved_3", "");
    pEle->SetAttribute("reserved_4", "");
    pEle->SetAttribute("reserved_5", "");
    pEle->SetAttribute("reserved_6", "");
    pEle->SetAttribute("reserved_7", "");
    pEle->SetAttribute("reserved_8", "");
    pEle->SetAttribute("reserved_9", "");
    pEle->SetAttribute("reserved_10", "");
    pDestCharge->LinkEndChild(pEle);

    pCfgDoc->SaveFile(strFileName);

    if(NULL != pCfgDoc)
    {
        delete pCfgDoc;
        pCfgDoc = NULL;
    }

    return true;
}

bool CConfiger::CreateVersionConfig(const char* strFileName)
{
    if(NULL == strFileName)
        return false;

    TiXmlDocument* pCfgDoc = new TiXmlDocument();

    TiXmlDeclaration* decl = new TiXmlDeclaration( "1.0", "utf-8", "" );
    pCfgDoc->LinkEndChild(decl);

    TiXmlElement* pEle = NULL;
    TiXmlElement* pRoot = new TiXmlElement("Root");
    pCfgDoc->LinkEndChild(pRoot);

    //version
    TiXmlElement* pDest = new TiXmlElement("DEST");
    pDest->SetAttribute("TYPE", "version");
    pRoot->LinkEndChild(pDest);

    pEle = new TiXmlElement("SYSTEM_VERSION");
    pEle->SetAttribute("system_version", "");
    pDest->LinkEndChild(pEle);

    pEle = new TiXmlElement("DRIVER_VERSION");
    pEle->SetAttribute("driver_version", "");
    pDest->LinkEndChild(pEle);

    pEle = new TiXmlElement("ULTRA_VERSION");
    pEle->SetAttribute("ultra_version", "");
    pDest->LinkEndChild(pEle);

    pEle = new TiXmlElement("CHARGE_VERSION");
    pEle->SetAttribute("charge_version", "");
    pDest->LinkEndChild(pEle);

    pEle = new TiXmlElement("RESERVED");
    pEle->SetAttribute("reserved_1", "");
    pEle->SetAttribute("reserved_2", "");
    pEle->SetAttribute("reserved_3", "");
    pEle->SetAttribute("reserved_4", "");
    pEle->SetAttribute("reserved_5", "");
    pEle->SetAttribute("reserved_6", "");
    pEle->SetAttribute("reserved_7", "");
    pEle->SetAttribute("reserved_8", "");
    pEle->SetAttribute("reserved_9", "");
    pEle->SetAttribute("reserved_10", "");
    pDest->LinkEndChild(pEle);

    pCfgDoc->SaveFile(strFileName);

    if(NULL != pCfgDoc)
    {
        delete pCfgDoc;
        pCfgDoc = NULL;
    }

    return true;
}

bool CConfiger::ReadDriverConfig(const char* strFileName, DRIVER_CONFIG* pConfigData)
{
    if(NULL == strFileName || NULL == pConfigData)
        return false;

    TiXmlDocument tinyDoc;
    if(!tinyDoc.LoadFile(std::string(strFileName)))
    {
        CreateDriverConfig(strFileName);
        return false;
    }

    TiXmlElement* pRoot = tinyDoc.RootElement();
    if(NULL == pRoot)
    {
        CreateDriverConfig(strFileName);
        return false;
    }

    bool bFlag = false;
    TiXmlElement* pEle = pRoot->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->FirstAttribute()->Value(), "driver"))
        {
            bFlag = true;
            break;
        }

        pEle = pEle->NextSiblingElement();
    }

    if(!bFlag)
    {
        CreateDriverConfig(strFileName);
        return false;
    }

    double value = 0.0;
    pEle = pEle->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->Value(), STR_NODE_CONTROL_MODIFY))
        {
            pConfigData->control_modify.left_wheel_forward = atof(pEle->Attribute(STR_ATTR_LEFT_WHEEL_FORWARD, &value));
            pConfigData->control_modify.left_wheel_backward = atof(pEle->Attribute(STR_ATTR_LEFT_WHEEL_BACKWARD, &value));
            pConfigData->control_modify.right_wheel_forward = atof(pEle->Attribute(STR_ATTR_RIGHT_WHEEL_FORWARD, &value));
            pConfigData->control_modify.right_wheel_backward = atof(pEle->Attribute(STR_ATTR_RIGHT_WHEEL_BACKWARD, &value));
        }

        if(0 == strcmp(pEle->Value(), STR_NODE_CODES_TO_DISTANCE))
        {
            pConfigData->codes_to_distance.cofficient_modify = atof(pEle->Attribute(STR_ATTR_COFFICIENT_MODIFY, &value));
            pConfigData->codes_to_distance.driver_servo_interval = atof(pEle->Attribute(STR_ATTR_DRIVER_SERVO_INTERVAL, &value));
        }

        if(0 == strcmp(pEle->Value(), STR_NODE_DEAD_RECKON_COMPOENSATE))
        {
            pConfigData->dead_reckon_modify.left_wheel_forward = atof(pEle->Attribute(STR_ATTR_LEFT_WHEEL_FORWARD, &value));
            pConfigData->dead_reckon_modify.left_wheel_backward = atof(pEle->Attribute(STR_ATTR_LEFT_WHEEL_BACKWARD, &value));
            pConfigData->dead_reckon_modify.right_wheel_forward = atof(pEle->Attribute(STR_ATTR_RIGHT_WHEEL_FORWARD, &value));
            pConfigData->dead_reckon_modify.right_wheel_backward = atof(pEle->Attribute(STR_ATTR_RIGHT_WHEEL_BACKWARD, &value));
        }

        if(0 == strcmp(pEle->Value(), STR_NODE_WHEELS_SPACES_MODIFY))
            pConfigData->wheels_spaces_modify = atof(pEle->Attribute(STR_ATTR_WHEELS_SPACES_MODIFY, &value));

        pEle = pEle->NextSiblingElement();
    }

    return true;
}

bool CConfiger::SetDriverConfig(const char* strFileName, DRIVER_CONFIG* pConfigData)
{
    if(NULL == strFileName || NULL == pConfigData)
        return false;

    TiXmlDocument tinyDoc;
    if(!tinyDoc.LoadFile(std::string(strFileName)))
    {
        CreateDriverConfig(strFileName);
        return false;
    }

    TiXmlElement* pRoot = tinyDoc.RootElement();
    if(NULL == pRoot)
    {
        CreateDriverConfig(strFileName);
        return false;
    }

    bool bFlag = false;
    TiXmlElement* pEle = pRoot->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->FirstAttribute()->Value(), "driver"))
        {
            bFlag = true;
            break;
        }

        pEle = pEle->NextSiblingElement();
    }

    if(!bFlag)
    {
        CreateDriverConfig(strFileName);
        return false;
    }

    pEle = pEle->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->Value(), STR_NODE_CONTROL_MODIFY))
        {
            pEle->SetDoubleAttribute(STR_ATTR_LEFT_WHEEL_FORWARD, pConfigData->control_modify.left_wheel_forward);
            pEle->SetDoubleAttribute(STR_ATTR_LEFT_WHEEL_BACKWARD, pConfigData->control_modify.left_wheel_backward);
            pEle->SetDoubleAttribute(STR_ATTR_RIGHT_WHEEL_FORWARD, pConfigData->control_modify.right_wheel_forward);
            pEle->SetDoubleAttribute(STR_ATTR_RIGHT_WHEEL_BACKWARD, pConfigData->control_modify.right_wheel_backward);
        }

        if(0 == strcmp(pEle->Value(), STR_NODE_CODES_TO_DISTANCE))
        {
            pEle->SetDoubleAttribute(STR_ATTR_COFFICIENT_MODIFY, pConfigData->codes_to_distance.cofficient_modify);
            pEle->SetDoubleAttribute(STR_ATTR_DRIVER_SERVO_INTERVAL, pConfigData->codes_to_distance.driver_servo_interval);
        }

        if(0 == strcmp(pEle->Value(), STR_NODE_DEAD_RECKON_COMPOENSATE))
        {
            pEle->SetDoubleAttribute(STR_ATTR_LEFT_WHEEL_FORWARD, pConfigData->dead_reckon_modify.left_wheel_forward);
            pEle->SetDoubleAttribute(STR_ATTR_LEFT_WHEEL_BACKWARD, pConfigData->dead_reckon_modify.left_wheel_backward);
            pEle->SetDoubleAttribute(STR_ATTR_RIGHT_WHEEL_FORWARD, pConfigData->dead_reckon_modify.right_wheel_forward);
            pEle->SetDoubleAttribute(STR_ATTR_RIGHT_WHEEL_BACKWARD, pConfigData->dead_reckon_modify.right_wheel_backward);
        }

        if(0 == strcmp(pEle->Value(), STR_NODE_WHEELS_SPACES_MODIFY))
            pEle->SetDoubleAttribute(STR_ATTR_WHEELS_SPACES_MODIFY, pConfigData->wheels_spaces_modify);

        pEle = pEle->NextSiblingElement();
    }

    tinyDoc.SaveFile(std::string(strFileName));

    return true;
}

bool CConfiger::ReadWorkConfig(const char* strFileName, WORK_CONFIG* pConfigData)
{
    if(NULL == strFileName || NULL == pConfigData)
        return false;

    TiXmlDocument tinyDoc;
    if(!tinyDoc.LoadFile(std::string(strFileName)))
        return false;

    TiXmlElement* pRoot = tinyDoc.RootElement();
    if(NULL == pRoot)
        return false;

    bool bFlag = false;
    TiXmlElement* pEle = pRoot->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->FirstAttribute()->Value(), "local"))
        {
            bFlag = true;
            break;
        }

        pEle = pEle->NextSiblingElement();
    }

    if(!bFlag)
        return false;

    int value_local = 0;
    pEle = pEle->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->Value(), STR_NODE_FAN))
            pConfigData->fan_time = atoi(pEle->Attribute(STR_ATTR_FAN_TIME, &value_local));

        if(0 == strcmp(pEle->Value(), STR_NODE_SERIAL))
            strcpy(pConfigData->serial_num, pEle->Attribute((STR_ATTR_SERIAL_NUM)));

        if(0 == strcmp(pEle->Value(), STR_NODE_CHARGE_ID))
            pConfigData->charge_pile_id = atoi(pEle->Attribute(STR_ATTR_CHARGE_ID, &value_local));

        if(0 == strcmp(pEle->Value(), STR_NODE_CHARGE_TYPE))
            pConfigData->charge_type = atoi(pEle->Attribute(STR_ATTR_CHARGE_TPYE, &value_local));

        if(0 == strcmp(pEle->Value(), STR_NODE_CHARGE_VER))
            strcpy(pConfigData->charge_pile_ver, pEle->Attribute(STR_ATTR_CHARGE_VER));

        if(0 == strcmp(pEle->Value(), STR_NODE_CHARGE_CHANNEL))
            pConfigData->charge_channel = atoi(pEle->Attribute(STR_ATTR_CHARGE_CHANNEL, &value_local));

        if(0 == strcmp(pEle->Value(), STR_NODE_INFRARED))
        {
            pConfigData->infrared_init[0] = atoi(pEle->Attribute(STR_ATTR_INFRARED_1, &value_local));
            pConfigData->infrared_init[1] = atoi(pEle->Attribute(STR_ATTR_INFRARED_2, &value_local));
            pConfigData->infrared_init[2] = atoi(pEle->Attribute(STR_ATTR_INFRARED_3, &value_local));
            pConfigData->infrared_init[3] = atoi(pEle->Attribute(STR_ATTR_INFRARED_4, &value_local));
            pConfigData->infrared_init[4] = atoi(pEle->Attribute(STR_ATTR_INFRARED_5, &value_local));
            pConfigData->infrared_init[5] = atoi(pEle->Attribute(STR_ATTR_INFRARED_6, &value_local));
            pConfigData->infrared_init[6] = atoi(pEle->Attribute(STR_ATTR_INFRARED_7, &value_local));
        }

        pEle = pEle->NextSiblingElement();
    }

    return true;
}

bool CConfiger::SetWorkConfig(const char* strFileName, WORK_CONFIG* pConfigData)
{
    if(NULL == strFileName || NULL == pConfigData)
        return false;

    TiXmlDocument tinyDoc;
    if(!tinyDoc.LoadFile(std::string(strFileName)))
        return false;

    TiXmlElement* pRoot = tinyDoc.RootElement();
    if(NULL == pRoot)
        return false;

    bool bFlag = false;
    TiXmlElement* pEle = pRoot->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->FirstAttribute()->Value(), "local"))
        {
            bFlag = true;
            break;
        }

        pEle = pEle->NextSiblingElement();
    }

    if(!bFlag)
        return false;

    pEle = pEle->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->Value(), STR_NODE_FAN))
            pEle->SetAttribute(STR_ATTR_FAN_TIME, pConfigData->fan_time);

        if(0 == strcmp(pEle->Value(), STR_NODE_SERIAL))
            pEle->SetAttribute(STR_ATTR_SERIAL_NUM, pConfigData->serial_num);

        if(0 == strcmp(pEle->Value(), STR_NODE_CHARGE_ID))
            pEle->SetAttribute(STR_ATTR_CHARGE_ID, pConfigData->charge_pile_id);

        if(0 == strcmp(pEle->Value(), STR_NODE_CHARGE_TYPE))
            pEle->SetAttribute(STR_ATTR_CHARGE_TPYE, pConfigData->charge_type);

        if(0 == strcmp(pEle->Value(), STR_NODE_CHARGE_VER))
            pEle->SetAttribute(STR_ATTR_CHARGE_VER, pConfigData->charge_pile_ver);

        if(0 == strcmp(pEle->Value(), STR_NODE_CHARGE_CHANNEL))
            pEle->SetAttribute(STR_ATTR_CHARGE_CHANNEL, pConfigData->charge_channel);

        if(0 == strcmp(pEle->Value(), STR_NODE_INFRARED))
        {
            pEle->SetAttribute(STR_ATTR_INFRARED_1, pConfigData->infrared_init[0]);
            pEle->SetAttribute(STR_ATTR_INFRARED_2, pConfigData->infrared_init[1]);
            pEle->SetAttribute(STR_ATTR_INFRARED_3, pConfigData->infrared_init[2]);
            pEle->SetAttribute(STR_ATTR_INFRARED_4, pConfigData->infrared_init[3]);
            pEle->SetAttribute(STR_ATTR_INFRARED_5, pConfigData->infrared_init[4]);
            pEle->SetAttribute(STR_ATTR_INFRARED_6, pConfigData->infrared_init[5]);
            pEle->SetAttribute(STR_ATTR_INFRARED_7, pConfigData->infrared_init[6]);
        }

        pEle = pEle->NextSiblingElement();
    }
    tinyDoc.SaveFile(std::string(strFileName));

    return true;
}

bool CConfiger::ReadLocalConfig(const char* strFileName, LOCAL_CONFIG* pConfigData)
{
    if(NULL == strFileName || NULL == pConfigData)
        return false;

    TiXmlDocument tinyDoc;
    if(!tinyDoc.LoadFile(std::string(strFileName)))
    {
        CreateLocalConfig(strFileName);
        return false;
    }

    TiXmlElement* pRoot = tinyDoc.RootElement();
    if(NULL == pRoot)
    {
        CreateLocalConfig(strFileName);
        return false;
    }

    bool bFlag = false;
    TiXmlElement* pEle = pRoot->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->FirstAttribute()->Value(), "local"))
        {
            bFlag = true;
            break;
        }

        pEle = pEle->NextSiblingElement();
    }

    if(!bFlag)
    {
        CreateLocalConfig(strFileName);
        return false;
    }

    int value_local = 0;
    pEle = pEle->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->Value(), STR_NODE_FAN))
            pConfigData->fan_time = atoi(pEle->Attribute(STR_ATTR_FAN_TIME, &value_local));

        if(0 == strcmp(pEle->Value(), STR_NODE_SERIAL))
            strcpy(pConfigData->serial_num, pEle->Attribute((STR_ATTR_SERIAL_NUM)));

        if(0 == strcmp(pEle->Value(), STR_NODE_INFRARED))
        {
            pConfigData->infrared_init[0] = atoi(pEle->Attribute(STR_ATTR_INFRARED_1, &value_local));
            pConfigData->infrared_init[1] = atoi(pEle->Attribute(STR_ATTR_INFRARED_2, &value_local));
            pConfigData->infrared_init[2] = atoi(pEle->Attribute(STR_ATTR_INFRARED_3, &value_local));
            pConfigData->infrared_init[3] = atoi(pEle->Attribute(STR_ATTR_INFRARED_4, &value_local));
            pConfigData->infrared_init[4] = atoi(pEle->Attribute(STR_ATTR_INFRARED_5, &value_local));
            pConfigData->infrared_init[5] = atoi(pEle->Attribute(STR_ATTR_INFRARED_6, &value_local));
            pConfigData->infrared_init[6] = atoi(pEle->Attribute(STR_ATTR_INFRARED_7, &value_local));
        }

        pEle = pEle->NextSiblingElement();
    }

    return true;
}

bool CConfiger::SetLocalConfig(const char* strFileName, LOCAL_CONFIG* pConfigData)
{
    if(NULL == strFileName || NULL == pConfigData)
        return false;

    TiXmlDocument tinyDoc;
    if(!tinyDoc.LoadFile(std::string(strFileName)))
    {
        CreateLocalConfig(strFileName);
        return false;
    }

    TiXmlElement* pRoot = tinyDoc.RootElement();
    if(NULL == pRoot)
    {
        CreateLocalConfig(strFileName);
        return false;
    }

    bool bFlag = false;
    TiXmlElement* pEle = pRoot->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->FirstAttribute()->Value(), "local"))
        {
            bFlag = true;
            break;
        }

        pEle = pEle->NextSiblingElement();
    }

    if(!bFlag)
    {
        CreateLocalConfig(strFileName);
        return false;
    }

    pEle = pEle->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->Value(), STR_NODE_FAN))
            pEle->SetAttribute(STR_ATTR_FAN_TIME, pConfigData->fan_time);

        if(0 == strcmp(pEle->Value(), STR_NODE_SERIAL))
            pEle->SetAttribute(STR_ATTR_SERIAL_NUM, pConfigData->serial_num);

        if(0 == strcmp(pEle->Value(), STR_NODE_INFRARED))
        {
            pEle->SetAttribute(STR_ATTR_INFRARED_1, pConfigData->infrared_init[0]);
            pEle->SetAttribute(STR_ATTR_INFRARED_2, pConfigData->infrared_init[1]);
            pEle->SetAttribute(STR_ATTR_INFRARED_3, pConfigData->infrared_init[2]);
            pEle->SetAttribute(STR_ATTR_INFRARED_4, pConfigData->infrared_init[3]);
            pEle->SetAttribute(STR_ATTR_INFRARED_5, pConfigData->infrared_init[4]);
            pEle->SetAttribute(STR_ATTR_INFRARED_6, pConfigData->infrared_init[5]);
            pEle->SetAttribute(STR_ATTR_INFRARED_7, pConfigData->infrared_init[6]);
        }

        pEle = pEle->NextSiblingElement();
    }
    tinyDoc.SaveFile(std::string(strFileName));

    return true;
}

bool CConfiger::ReadChargeConfig(const char* strFileName, CHARGE_CONFIG* pConfigData)
{
    if(NULL == strFileName || NULL == pConfigData)
        return false;

    TiXmlDocument tinyDoc;
    if(!tinyDoc.LoadFile(std::string(strFileName)))
    {
        CreateChargeConfig(strFileName);
        return false;
    }

    TiXmlElement* pRoot = tinyDoc.RootElement();
    if(NULL == pRoot)
    {
        CreateChargeConfig(strFileName);
        return false;
    }

    bool bFlag = false;
    TiXmlElement* pEle = pRoot->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->FirstAttribute()->Value(), "charge"))
        {
            bFlag = true;
            break;
        }

        pEle = pEle->NextSiblingElement();
    }

    if(!bFlag)
    {
        CreateChargeConfig(strFileName);
        return false;
    }

    int value_local = 0;
    pEle = pEle->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->Value(), STR_NODE_CHARGE_ID))
            pConfigData->charge_pile_id = atoi(pEle->Attribute(STR_ATTR_CHARGE_ID, &value_local));

        if(0 == strcmp(pEle->Value(), STR_NODE_CHARGE_TYPE))
            pConfigData->charge_type = atoi(pEle->Attribute(STR_ATTR_CHARGE_TYPE, &value_local));

        if(0 == strcmp(pEle->Value(), STR_NODE_CHARGE_VER))
            strcpy(pConfigData->charge_pile_ver, pEle->Attribute(STR_ATTR_CHARGE_VER));

        if(0 == strcmp(pEle->Value(), STR_NODE_CHARGE_CHANNEL))
            pConfigData->charge_channel = atoi(pEle->Attribute(STR_ATTR_CHARGE_CHANNEL, &value_local));

        pEle = pEle->NextSiblingElement();
    }
    return true;
}

bool CConfiger::SetChargeConfig(const char* strFileName, CHARGE_CONFIG* pConfigData)
{
    if(NULL == strFileName || NULL == pConfigData)
        return false;

    TiXmlDocument tinyDoc;
    if(!tinyDoc.LoadFile(std::string(strFileName)))
    {
        CreateChargeConfig(strFileName);
        return false;
    }

    TiXmlElement* pRoot = tinyDoc.RootElement();
    if(NULL == pRoot)
    {
        CreateChargeConfig(strFileName);
        return false;
    }

    bool bFlag = false;
    TiXmlElement* pEle = pRoot->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->FirstAttribute()->Value(), "charge"))
        {
            bFlag = true;
            break;
        }

        pEle = pEle->NextSiblingElement();
    }

    if(!bFlag)
    {
        CreateChargeConfig(strFileName);
        return false;
    }

    pEle = pEle->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->Value(), STR_NODE_CHARGE_ID))
            pEle->SetAttribute(STR_ATTR_CHARGE_ID, pConfigData->charge_pile_id);

        if(0 == strcmp(pEle->Value(), STR_NODE_CHARGE_TYPE))
            pEle->SetAttribute(STR_ATTR_CHARGE_TYPE, pConfigData->charge_type);

        if(0 == strcmp(pEle->Value(), STR_NODE_CHARGE_VER))
            pEle->SetAttribute(STR_ATTR_CHARGE_VER, pConfigData->charge_pile_ver);

        if(0 == strcmp(pEle->Value(), STR_NODE_CHARGE_CHANNEL))
            pEle->SetAttribute(STR_ATTR_CHARGE_CHANNEL, pConfigData->charge_channel);


        pEle = pEle->NextSiblingElement();
    }

    tinyDoc.SaveFile(std::string(strFileName));
    return true;
}

bool CConfiger::ReadVersionConfig(const char* strFileName, VERSION_CONFIG* pConfigData)
{
    if(NULL == strFileName || NULL == pConfigData)
        return false;

    TiXmlDocument tinyDoc;
    if(!tinyDoc.LoadFile(std::string(strFileName)))
    {
        CreateVersionConfig(strFileName);
        return false;
    }

    TiXmlElement* pRoot = tinyDoc.RootElement();
    if(NULL == pRoot)
    {
        CreateVersionConfig(strFileName);
        return false;
    }

    bool bFlag = false;
    TiXmlElement* pEle = pRoot->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->FirstAttribute()->Value(), "version"))
        {
            bFlag = true;
            break;
        }

        pEle = pEle->NextSiblingElement();
    }

    if(!bFlag)
    {
        CreateVersionConfig(strFileName);
        return false;
    }

    pEle = pEle->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->Value(), STR_NODE_SYS_VERSION))
            strcpy(pConfigData->sys_version, pEle->Attribute((STR_ATTR_SYS_VERSION)));

        if(0 == strcmp(pEle->Value(), STR_NODE_DRIVER_VERSION))
            strcpy(pConfigData->driver_version, pEle->Attribute((STR_ATTR_DRIVER_VERSION)));

        if(0 == strcmp(pEle->Value(), STR_NODE_ULTRA_VERSION))
            strcpy(pConfigData->ultra_version, pEle->Attribute((STR_ATTR_ULTRA_VERSION)));

        if(0 == strcmp(pEle->Value(), STR_NODE_CHARGE_VERSION))
            strcpy(pConfigData->charge_version, pEle->Attribute((STR_ATTR_CHARGE_VERSION)));

        pEle = pEle->NextSiblingElement();
    }
    return true;
}


bool CConfiger::SetVersionConfig(const char* strFileName, VERSION_CONFIG* pConfigData)
{
    if(NULL == strFileName || NULL == pConfigData)
        return false;

    TiXmlDocument tinyDoc;
    if(!tinyDoc.LoadFile(std::string(strFileName)))
    {
        CreateVersionConfig(strFileName);
        return false;
    }

    TiXmlElement* pRoot = tinyDoc.RootElement();
    if(NULL == pRoot)
    {
        CreateVersionConfig(strFileName);
        return false;
    }

    bool bFlag = false;
    TiXmlElement* pEle = pRoot->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->FirstAttribute()->Value(), "version"))
        {
            bFlag = true;
            break;
        }

        pEle = pEle->NextSiblingElement();
    }

    if(!bFlag)
    {
        CreateVersionConfig(strFileName);
        return false;
    }

    pEle = pEle->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->Value(), STR_NODE_SYS_VERSION))
            pEle->SetAttribute(STR_ATTR_SYS_VERSION, pConfigData->sys_version);

        if(0 == strcmp(pEle->Value(), STR_NODE_DRIVER_VERSION))
            pEle->SetAttribute(STR_ATTR_DRIVER_VERSION, pConfigData->driver_version);

        if(0 == strcmp(pEle->Value(), STR_NODE_ULTRA_VERSION))
            pEle->SetAttribute(STR_ATTR_ULTRA_VERSION, pConfigData->ultra_version);

        if(0 == strcmp(pEle->Value(), STR_NODE_CHARGE_VERSION))
            pEle->SetAttribute(STR_ATTR_CHARGE_VERSION, pConfigData->charge_version);

        pEle = pEle->NextSiblingElement();
    }

    tinyDoc.SaveFile(std::string(strFileName));

    return true;
}

bool CConfiger::ReadLogConfig(const char* strFileName, NodeCommonInfo* nci)
{
    if(NULL == strFileName)
        return false;

    TiXmlDocument tinyDoc;
    if(!tinyDoc.LoadFile(std::string(strFileName)))
    {
        return false;
    }

    TiXmlElement* pRoot = tinyDoc.RootElement();
    if(NULL == pRoot)
    {
        return false;
    }

    bool bFlag = false;
    TiXmlElement* pEle = pRoot->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->FirstAttribute()->Value(), "log"))
        {
            bFlag = true;
            break;
        }

        pEle = pEle->NextSiblingElement();
    }

    if(!bFlag)
    {
        return false;
    }

    pEle = pEle->FirstChildElement();
    while(pEle)
    {
    	int index = 0;
        if(0 == strcmp(pEle->Value(), STR_NODE_CAMERAOLD))
        {
        	index = atoi(pEle->Attribute(STR_ATTR_LOG_INDEX));
        	strcpy(nci[index-1].node_name, pEle->Attribute(STR_ATTR_LOG_NAME));
            nci[index-1].log_site = atoi(pEle->Attribute(STR_ATTR_LOG_SITE));
            nci[index-1].log_level = atoi(pEle->Attribute(STR_ATTR_LOG_LEVEL));
            nci[index-1].modify = atoi(pEle->Attribute(STR_ATTR_LOG_MODIFY));
        }

        else if(0 == strcmp(pEle->Value(), STR_NODE_CONFIGOLD))
        {
        	index = atoi(pEle->Attribute(STR_ATTR_LOG_INDEX));
        	strcpy(nci[index-1].node_name, pEle->Attribute(STR_ATTR_LOG_NAME));
            nci[index-1].log_site = atoi(pEle->Attribute(STR_ATTR_LOG_SITE));
            nci[index-1].log_level = atoi(pEle->Attribute(STR_ATTR_LOG_LEVEL));
            nci[index-1].modify = atoi(pEle->Attribute(STR_ATTR_LOG_MODIFY));
        }

        else if(0 == strcmp(pEle->Value(), STR_NODE_CORESERIALOLD))
        {
        	index = atoi(pEle->Attribute(STR_ATTR_LOG_INDEX));
        	strcpy(nci[index-1].node_name, pEle->Attribute(STR_ATTR_LOG_NAME));
            nci[index-1].log_site = atoi(pEle->Attribute(STR_ATTR_LOG_SITE));
            nci[index-1].log_level = atoi(pEle->Attribute(STR_ATTR_LOG_LEVEL));
            nci[index-1].modify = atoi(pEle->Attribute(STR_ATTR_LOG_MODIFY));
        }

        else if(0 == strcmp(pEle->Value(), STR_NODE_INFRAREDOLD))
        {
        	index = atoi(pEle->Attribute(STR_ATTR_LOG_INDEX));
        	strcpy(nci[index-1].node_name, pEle->Attribute(STR_ATTR_LOG_NAME));
            nci[index-1].log_site = atoi(pEle->Attribute(STR_ATTR_LOG_SITE));
            nci[index-1].log_level = atoi(pEle->Attribute(STR_ATTR_LOG_LEVEL));
            nci[index-1].modify = atoi(pEle->Attribute(STR_ATTR_LOG_MODIFY));
        }

        else if(0 == strcmp(pEle->Value(), STR_NODE_IODETECTIONOLD))
        {
        	index = atoi(pEle->Attribute(STR_ATTR_LOG_INDEX));
        	strcpy(nci[index-1].node_name, pEle->Attribute(STR_ATTR_LOG_NAME));
            nci[index-1].log_site = atoi(pEle->Attribute(STR_ATTR_LOG_SITE));
            nci[index-1].log_level = atoi(pEle->Attribute(STR_ATTR_LOG_LEVEL));
            nci[index-1].modify = atoi(pEle->Attribute(STR_ATTR_LOG_MODIFY));
        }

        else if(0 == strcmp(pEle->Value(), STR_NODE_LOGOLD))
        {
        	index = atoi(pEle->Attribute(STR_ATTR_LOG_INDEX));
        	strcpy(nci[index-1].node_name, pEle->Attribute(STR_ATTR_LOG_NAME));
            nci[index-1].log_site = atoi(pEle->Attribute(STR_ATTR_LOG_SITE));
            nci[index-1].log_level = atoi(pEle->Attribute(STR_ATTR_LOG_LEVEL));
            nci[index-1].modify = atoi(pEle->Attribute(STR_ATTR_LOG_MODIFY));
        }

        else if(0 == strcmp(pEle->Value(), STR_NODE_MOTIONOLD))
        {
        	index = atoi(pEle->Attribute(STR_ATTR_LOG_INDEX));
        	strcpy(nci[index-1].node_name, pEle->Attribute(STR_ATTR_LOG_NAME));
            nci[index-1].log_site = atoi(pEle->Attribute(STR_ATTR_LOG_SITE));
            nci[index-1].log_level = atoi(pEle->Attribute(STR_ATTR_LOG_LEVEL));
            nci[index-1].modify = atoi(pEle->Attribute(STR_ATTR_LOG_MODIFY));
        }

        else if(0 == strcmp(pEle->Value(), STR_NODE_POWERDETECTIONOLD))
        {
        	index = atoi(pEle->Attribute(STR_ATTR_LOG_INDEX));
        	strcpy(nci[index-1].node_name, pEle->Attribute(STR_ATTR_LOG_NAME));
            nci[index-1].log_site = atoi(pEle->Attribute(STR_ATTR_LOG_SITE));
            nci[index-1].log_level = atoi(pEle->Attribute(STR_ATTR_LOG_LEVEL));
            nci[index-1].modify = atoi(pEle->Attribute(STR_ATTR_LOG_MODIFY));
        }

        else if(0 == strcmp(pEle->Value(), STR_NODE_PROTECTOLD))
        {
        	index = atoi(pEle->Attribute(STR_ATTR_LOG_INDEX));
        	strcpy(nci[index-1].node_name, pEle->Attribute(STR_ATTR_LOG_NAME));
            nci[index-1].log_site = atoi(pEle->Attribute(STR_ATTR_LOG_SITE));
            nci[index-1].log_level = atoi(pEle->Attribute(STR_ATTR_LOG_LEVEL));
            nci[index-1].modify = atoi(pEle->Attribute(STR_ATTR_LOG_MODIFY));
        }

        else if(0 == strcmp(pEle->Value(), STR_NODE_RECALIBRATIONOLD))
        {
        	index = atoi(pEle->Attribute(STR_ATTR_LOG_INDEX));
        	strcpy(nci[index-1].node_name, pEle->Attribute(STR_ATTR_LOG_NAME));
            nci[index-1].log_site = atoi(pEle->Attribute(STR_ATTR_LOG_SITE));
            nci[index-1].log_level = atoi(pEle->Attribute(STR_ATTR_LOG_LEVEL));
            nci[index-1].modify = atoi(pEle->Attribute(STR_ATTR_LOG_MODIFY));
        }

        else if(0 == strcmp(pEle->Value(), STR_NODE_ULTRASONICOLD))
        {
        	index = atoi(pEle->Attribute(STR_ATTR_LOG_INDEX));
        	strcpy(nci[index-1].node_name, pEle->Attribute(STR_ATTR_LOG_NAME));
            nci[index-1].log_site = atoi(pEle->Attribute(STR_ATTR_LOG_SITE));
            nci[index-1].log_level = atoi(pEle->Attribute(STR_ATTR_LOG_LEVEL));
            nci[index-1].modify = atoi(pEle->Attribute(STR_ATTR_LOG_MODIFY));
        }

        else if(0 == strcmp(pEle->Value(), STR_NODE_UPGRADEOLD))
        {
        	index = atoi(pEle->Attribute(STR_ATTR_LOG_INDEX));
        	strcpy(nci[index-1].node_name, pEle->Attribute(STR_ATTR_LOG_NAME));
            nci[index-1].log_site = atoi(pEle->Attribute(STR_ATTR_LOG_SITE));
            nci[index-1].log_level = atoi(pEle->Attribute(STR_ATTR_LOG_LEVEL));
            nci[index-1].modify = atoi(pEle->Attribute(STR_ATTR_LOG_MODIFY));
        }

        pEle = pEle->NextSiblingElement();
    }
	return true;
}

bool CConfiger::SetLogConfig(const char* strFileName, NodeCommonInfo* pNci)
{
	return true;
}

bool CConfiger::ReadSerialConfig(const char* strFileName, SerialPortInfo* pSpi)
{
    if(NULL == strFileName)
        return false;

    TiXmlDocument tinyDoc;
    if(!tinyDoc.LoadFile(std::string(strFileName)))
    {
        return false;
    }

    TiXmlElement* pRoot = tinyDoc.RootElement();
    if(NULL == pRoot)
    {
        return false;
    }

    bool bFlag = false;
    TiXmlElement* pEle = pRoot->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->FirstAttribute()->Value(), "serial"))
        {
            bFlag = true;
            break;
        }

        pEle = pEle->NextSiblingElement();
    }

    if(!bFlag)
    {
        return false;
    }

    pEle = pEle->FirstChildElement();
    while(pEle)
    {
        if(0 == strcmp(pEle->Value(), STR_SERL_PAD_NAME))
        {
        	strcpy(pSpi->pad_read, pEle->Attribute(STR_SERL_PORT_READ));
        	strcpy(pSpi->pad_write, pEle->Attribute(STR_SERL_PORT_WRITE));
        }
        if(0 == strcmp(pEle->Value(), STR_SERL_ULTRA_NAME))
        {
        	strcpy(pSpi->ultra_read, pEle->Attribute(STR_SERL_PORT_READ));
        	strcpy(pSpi->ultra_write, pEle->Attribute(STR_SERL_PORT_WRITE));
        }
        if(0 == strcmp(pEle->Value(), STR_SERL_MOTOR_NAME))
        {
        	strcpy(pSpi->motor_read, pEle->Attribute(STR_SERL_PORT_READ));
        	strcpy(pSpi->motor_write, pEle->Attribute(STR_SERL_PORT_WRITE));
        }

        pEle = pEle->NextSiblingElement();
    }

	return true;
}
