#include "MainWindow.h"
#include <gtkmm/application.h>
#include <gst/gst.h>

int main(int argc, char* argv[])
{
    // 1. Initialize GStreamer C API
    setenv("GDK_BACKEND", "x11", 0);
//    gst_init(&argc, &argv);

    auto app = Gtk::Application::create("org.gtkmm.RtpStreamer");
    Gst::init(argc, argv); // Note: Since gst_init is called, this may be redundant but worth testing.
    MainWindow window;
    
    return app->run(window);
}