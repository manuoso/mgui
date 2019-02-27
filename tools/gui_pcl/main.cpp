//---------------------------------------------------------------------------------------------------------------------
//  MGUI
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 Manuel Pérez Jiménez (a.k.a. manuoso) manuperezj@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#include <mgui/guis/pclviewer_gui.h>
#include <QApplication>

#ifdef MGUI_USE_ROS
    #include <ros/ros.h>
#endif

int main(int _argc, char** _argv) {
	
    if (_argc != 2) {
        std::cout << "Bad input arguments, please provide only the path of a json config file with the structure detailed in the documentation" << std::endl;
        return -1;
    }

    #ifdef MGUI_USE_ROS
        ros::init(_argc, _argv, "pclgui");
        ros::AsyncSpinner rosSpinner(4);
        rosSpinner.start();
    #endif
    
    QApplication a(_argc, _argv);

    PCLViewer_gui gui;
    gui.configureGUI(_argc, _argv);
    gui.show();
    
    return a.exec();
}

