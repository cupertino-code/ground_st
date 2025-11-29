#pragma once

#include <gtkmm.h>
#include <gdk/gdk.h>          // The base GDK C header
#include <gdk/gdkx.h>         // <-- ADD THIS for gdk_x11_window_get_xid
#include <gstreamermm.h> 
#include <gstreamermm/videooverlay.h>
#include <glibmm.h>
#include <sys/mman.h> // For mmap
#include <fcntl.h>    // For shm_open
#include <unistd.h>   // For close
#include <vector>
#include "DrawingAreas.h"
#include "common.h"
#include "config.h"
#include "main_update.h"

// Shared Memory and Path Configuration
#define SHARED_NAME "/channel_data"

class MainWindow : public Gtk::Window {
public:
    MainWindow();
    virtual ~MainWindow();
    void update_status(void);
    void set_status_line(int severity, const char *status);
protected:
    bool on_key_press_event(GdkEventKey* event) override;
private:
    // --- GUI Elements ---
    Gtk::VBox m_VBox;
    Gtk::HBox m_VideoAndDrawBox;
    Gtk::HBox m_ButtonBox;
    Gtk::Button m_ButtonRecord, m_ButtonPower, m_ButtonQuit;
    Gtk::DrawingArea m_VideoArea; // Area for video display
    Gtk::Grid m_StatusGrid;
    AntennaDrawArea m_DrawingArea;      // Top Drawing Area
    VRXDrawArea m_BottomDrawingArea; // <-- NEW: Bottom Drawing Area
    Gtk::Button m_btn_left_fast, m_btn_left, m_btn_right, m_btn_right_fast;
    // --- GStreamer Elements ---
    GstElement *m_pipeline = nullptr;
    GstElement *m_tee = nullptr;
    GstElement *m_videosink = nullptr;

    // Recording branch elements
    std::vector<GstElement*> m_record_elements;
    GstPad *m_tee_record_pad = nullptr;
    global_status_t m_status;
    app_config_t m_app_config;
    // --- Private Methods ---
    void setup_pipeline();
    void set_video_overlay();
    void show_help_dialog();

    // Button Handlers
    void on_record_clicked();
    void on_power_clicked();
    void on_quit_clicked();
    void on_left_fast_clicked();
    void on_left_clicked();
    void on_right_clicked();
    void on_right_fast_clicked();
    bool on_redraw_timeout();
    bool on_update_status(void);

    // Recording Logic
    void start_recording();
    void stop_recording();
    void cleanup_recording_branch();

    // GStreamer Message Bus
    bool on_bus_message(const Glib::RefPtr<Gst::Bus>& bus, const Glib::RefPtr<Gst::Message>& message);

    // Signal Handling (SIGUSR1, SIGINT/TERM)
    static void signal_handler(int signum);
    static void on_sigusr1();
};
