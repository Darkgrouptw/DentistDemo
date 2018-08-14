#include "MyForm.h"

namespace DentistDemo
{
    void MyForm::Callback_DeviceInitDone(System::String ^address)
    {
        Console::WriteLine("Device Init Done: " + address);

        // Transfer to State_Connection::ComOpened
        UpdateConnectionStatus(State_Connection::ComOpened);
    }

    void MyForm::Callback_DeviceCloseDone(System::String ^comport)
    {
        Console::WriteLine("Device Close Done: " + comport);

        // Transfer to State_Connection::DeviceReady
        UpdateConnectionStatus(State_Connection::DeviceReady);
    }

    void MyForm::Callback_DeviceDiscovered(DeviceInfo ^deviceInfo)
    {
        if (this->InvokeRequired)
        {
            this->Invoke(gcnew Callback_DeviceDiscoveredDelegate(this, &MyForm::Callback_DeviceDiscovered)
                , gcnew array<Object^> { deviceInfo });
        }
        else
        {
            bleDeviceList->Add(deviceInfo->DeviceAddress, deviceInfo);

            cbBleDeviceL->DataSource = gcnew BindingSource(bleDeviceList, nullptr);
            cbBleDeviceL->DisplayMember = "Value";
            cbBleDeviceL->ValueMember = "Key";

            //cbBleDeviceR->DataSource = gcnew BindingSource(bleDeviceList, nullptr);
            //cbBleDeviceR->DisplayMember = "Value";
            //cbBleDeviceR->ValueMember = "Key";
        }
    }

    void MyForm::Callback_EstablishLinkDone(DeviceInfo ^deviceInfo, unsigned char status)
    {
        if (status == (unsigned char)HCICmds::HCI_StatusCodes::Success) // Success
        {
            Console::WriteLine("Device Established: " + deviceInfo->DeviceAddress);

            if (System::String::Equals(deviceInfo->DeviceAddress, bleDeviceL->GetEstablishedDevice()->DeviceAddress))
            {
                // Send notify request
                Console::WriteLine("Notify: " + bleDeviceL->GetEstablishedDevice()->ToString());
                bleDeviceL->StartNotification();
            }
			//else if (String::Equals(deviceInfo->DeviceAddress, bleDeviceR->GetEstablishedDevice()->DeviceAddress))
			//{
			//	// Send notify request
			//	Console::WriteLine("Notify: " + bleDeviceR->GetEstablishedDevice()->ToString());
			//	bleDeviceR->StartNotification();
			//}
        }
        else
        {

        }
    }

    void MyForm::Callback_TerminateLinkDone(DeviceInfo ^deviceInfo, unsigned char status, bool byHost)
    {
        if (status == (unsigned char)HCICmds::HCI_StatusCodes::Success)
        {
            Console::WriteLine("Device Terminated: " + deviceInfo->DeviceAddress);
        }
        else
        {

        }

        // Transfer to State_Connection::ComOpened
        UpdateConnectionStatus(State_Connection::ComOpened);
    }

    void MyForm::Callback_HandleValueNotification(DeviceInfo ^deviceInfo, array<unsigned char> ^payload)
    {
        unsigned char type = DevUtils::GetPacketType(payload);
        if (bleDeviceL->GetEstablishedDevice() != nullptr && System::String::Equals(deviceInfo->DeviceAddress, bleDeviceL->GetEstablishedDevice()->DeviceAddress))
        {
            // From left hand
            if (type == 0x00)
            {
                Quaternion quat = DevUtils::GetQuaternion(payload, 1); // 直接拿手套資料
                if (preQuaternL == nullptr)
                {
                    preQuaternL = new Quaternion(quat);
                    (sysManager->GetGloveDataLPtr())->Zero();
                }
				//std::cout << quat << std::endl;
				// 這個是程式那個手套左手模型的quaternion設定
				//sysManager->GetGloveDataL().GetQuaternion();


				//(sysManager->GetGloveDataLPtr())->SetQuaternion(quat);

                (sysManager->GetGloveDataLPtr())->SetQuaternion(sysManager->GetGloveDataL().GetQuaternion() * (preQuaternL->Inverse() * quat));
                preQuaternL = new Quaternion(quat);
            }
            else if (type == 0x01)
            {
                (sysManager->GetGloveDataLPtr())->SetAcceleration(DevUtils::GetAcceleration(payload, 11));
                (sysManager->GetGloveDataLPtr())->SetFlexRaw(DevUtils::GetFingerFlex(payload, 1));
                (sysManager->GetGloveDataLPtr())->Update();
            }
        }
		//else if (bleDeviceR->GetEstablishedDevice() != nullptr && String::Equals(deviceInfo->DeviceAddress, bleDeviceR->GetEstablishedDevice()->DeviceAddress))
		//{
		//	// From right hand
		//	if (type == 0x00)
		//	{
		//		Quaternion quat = DevUtils::GetQuaternion(payload, 1);
		//		//if (preQuaternR == nullptr)
		//		//{
		//		//	preQuaternR = new Quaternion(quat);
		//		//	(sysManager->GetGloveDataRPtr())->Zero();
		//		//}
		//		//(sysManager->GetGloveDataRPtr())->SetQuaternion(sysManager->GetGloveDataR().GetQuaternion() * (preQuaternR->Inverse() * quat));
		//		//preQuaternR = new Quaternion(quat);
		//	}
		//	else if (type == 0x01)
		//	{
		//		(sysManager->GetGloveDataRPtr())->SetAcceleration(DevUtils::GetAcceleration(payload, 11));
		//		(sysManager->GetGloveDataRPtr())->SetFlexRaw(DevUtils::GetFingerFlex(payload, 1));
		//		(sysManager->GetGloveDataRPtr())->Update();
		//	}
		//}
    }

    void MyForm::Callback_HandleCheckNotifyStatus(DeviceInfo ^deviceInfo, bool isNotifying)
    {
        Console::WriteLine("Device Notify Status: " + deviceInfo->DeviceAddress + " " + isNotifying);

        if (isNotifying)
        {
            // Transfer to State_Connection::BleEstablished
            UpdateConnectionStatus(State_Connection::BleEstablished);
        }
        else
        {
            // Terminate connection
            if (mState_Connection == State_Connection::BleEstablished)
            {
                if (bleDeviceL->GetEstablishedDevice() != nullptr && System::String::Equals(deviceInfo->DeviceAddress, bleDeviceL->GetEstablishedDevice()->DeviceAddress))
                {
                    Console::WriteLine("Terminate: " + bleDeviceL->GetEstablishedDevice()->ToString());
                    bleDeviceL->Terminate();
                }
                //else if (bleDeviceR->GetEstablishedDevice() != nullptr && String::Equals(deviceInfo->DeviceAddress, bleDeviceR->GetEstablishedDevice()->DeviceAddress))
                //{
                //    Console::WriteLine("Terminate: " + bleDeviceR->GetEstablishedDevice()->ToString());
                //    bleDeviceR->Terminate();
                //}
            }
        }
    }

    void MyForm::Callback_HandleChangeNotifyStatus(DeviceInfo ^deviceInfo, bool isSuccess)
    {
        Console::WriteLine("Device Change Notify Status: " + deviceInfo->DeviceAddress + " " + isSuccess);

        if (isSuccess)
        {
            if (bleDeviceL->GetEstablishedDevice() != nullptr && System::String::Equals(deviceInfo->DeviceAddress, bleDeviceL->GetEstablishedDevice()->DeviceAddress))
            {
                // Send notify request
                Console::WriteLine("Check Notify: " + bleDeviceL->GetEstablishedDevice()->ToString());
                bleDeviceL->CheckNotificationStatus();
            }
            //else if (bleDeviceR->GetEstablishedDevice() != nullptr && String::Equals(deviceInfo->DeviceAddress, bleDeviceR->GetEstablishedDevice()->DeviceAddress))
            //{
            //   // Send notify request
            //    Console::WriteLine("Check Notify: " + bleDeviceR->GetEstablishedDevice()->ToString());
            //    bleDeviceR->CheckNotificationStatus();
            //}
        }
        else
        {
            // Nothing to do now...
        }
    }
}