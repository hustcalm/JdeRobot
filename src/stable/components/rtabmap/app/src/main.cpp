/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


/*
 *  Copyright (C) 1997-2015 JDE Developers Team
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/.
 *
 *  Author : Lihang Li<licalmer@gmail.com>

 */

#include <iostream>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <pthread.h>
#include <jderobot/camera.h>
#include <IceUtil/Thread.h>
#include "parallelIce/cameraClient.h"

#include <QApplication>
#include <QtCore/QDir>
#include "rtabmap/utilite/UEventsManager.h"
#include "rtabmap/core/RtabmapThread.h"
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/gui/MainWindow.h"
#include <QMessageBox>
#include "rtabmap/utilite/UObjDeletionThread.h"
#include "ObjDeletionHandler.h"

using namespace rtabmap;

jderobot::cameraClient* camRGB=NULL;
jderobot::cameraClient* camDEPTH=NULL;

int main(int argc, char** argv)
{
    int status;
    Ice::CommunicatorPtr ic;
    Ice::PropertiesPtr prop;

    pthread_attr_t attr;
    pthread_attr_init(&attr);

    try {
        ic = Ice::initialize(argc, argv);
        prop = ic->getProperties();
    } catch (const Ice::Exception& ex) {
        std::cerr << ex << std::endl;
        return 1;
    } catch (const char* msg) {
        std::cerr <<"Error:" << msg << std::endl;
        return 1;
    }

    if (prop->getPropertyAsIntWithDefault("rtabmap.CameraRGBActive", 0)) {
        camRGB = new jderobot::cameraClient(ic, "rtabmap.CameraRGB.");
        if (camRGB != NULL) {
            std::cout << "rtabmap: RGB Camera loaded." << std::endl;
        }
        else {
            throw "rtabmap: faild to load RGB Camera.";
        }
    }

    if (prop->getPropertyAsIntWithDefault("rtabmap.CameraDEPTHActive", 0)) {
        camDEPTH = new jderobot::cameraClient(ic, "rtabmap.CameraDEPTH.");
        if (camDEPTH != NULL) {
            std::cout << "rtabmap: DEPTH Camera loaded." << std::endl;
        }
        else {
            throw "rtabmap: failed to load DEPTH Camera.";
        }
    }

	/* Set logger type */
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

	/* Create tasks */
	QApplication * app = new QApplication(argc, argv);
	MainWindow * mainWindow = new MainWindow();

	UINFO("Program started...");

	UEventsManager::addHandler(mainWindow);

	/* Start thread's task */
	if(mainWindow->isSavedMaximized())
	{
		mainWindow->showMaximized();
	}
	else
	{
		mainWindow->show();
	}

	RtabmapThread * rtabmap = new RtabmapThread(new Rtabmap());
	rtabmap->start(); // start it not initialized... will be initialized by event from the gui
	UEventsManager::addHandler(rtabmap);

	// Now wait for application to finish
	app->connect( app, SIGNAL( lastWindowClosed() ),
				app, SLOT( quit() ) );
	app->exec();// MUST be called by the Main Thread

	/* Remove handlers */
	UEventsManager::removeHandler(mainWindow);
	UEventsManager::removeHandler(rtabmap);

	UINFO("Killing threads...");
	rtabmap->join(true);

	UINFO("Closing RTAB-Map...");
	delete rtabmap;
	delete mainWindow;
	delete app;
	UINFO("All done!");

    if (camRGB != NULL) {
        delete camRGB;
    }

    if (camDEPTH != NULL) {
        delete camDEPTH;
    }

    std::cout << "final" << std::endl;

    if (ic) {
        ic->destroy();
    }

    return status;
}
