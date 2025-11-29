#include "MainWindow.h"
#include <iostream>
#include <ctime>
#include <cstring>
#include <cstdlib> // For system()
#include <pthread.h>
#include "crsf2net.h"
#include "control.h"
#include "config.h"
#include "utils.h"

#define ENCODER_INCREMENT 2
// --- Global Pointer for Signal Handling ---
MainWindow *global_window_ptr = nullptr;
const int VIDEO_WIDTH = 720;
const int VIDEO_HEIGHT = 576;
#define CONFIG_FILE "vrxtbl.yaml"

// --- GStreamer Helper ---
GstElement* make_gst_element(const char* plugin, const char* name) {
    GstElement *element = gst_element_factory_make(plugin, name);
    if (!element) {
        std::cerr << "GStreamer Error: Element/plugin not found: " << plugin << std::endl;
    }
    return element;
}

// --- Signal Handling ---
void MainWindow::on_sigusr1()
{
    if (global_window_ptr) {
        std::cout << "\nToggling recording via SIGUSR1..." << std::endl;
        if (!global_window_ptr->m_status.recording) {
            global_window_ptr->start_recording();
        } else {
            global_window_ptr->stop_recording();
        }
    }
//    return false; // Executes once
}

void MainWindow::signal_handler(int signum)
{
    if (global_window_ptr) {
        if (signum == SIGUSR1) {
            // Defer execution to the main GLib loop
            Glib::signal_timeout().connect_once(sigc::ptr_fun(&MainWindow::on_sigusr1), 0);
        } else if (signum == SIGINT || signum == SIGTERM) {
            std::cout << "\nReceived termination signal. Shutting down..." << std::endl;
            Gtk::Main::quit();
        }
    }
}

// --- MainWindow Implementation ---
MainWindow::MainWindow() :
    m_ButtonRecord("Start Recording"),
    m_ButtonPower("Power OFF"),
    m_ButtonQuit("Quit")
{
    global_window_ptr = this;

    set_title("Ground FPV station");
    set_default_size(VIDEO_WIDTH+200, VIDEO_HEIGHT+200);

    memset(&m_status, 0, sizeof(m_status));
    pthread_mutex_init(&m_status.mutex, NULL);
    m_DrawingArea.set_status_ptr(&m_status);
    m_BottomDrawingArea.set_status_ptr(&m_status);
    // Setup Signals
    signal(SIGUSR1, signal_handler);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    set_default_size(VIDEO_WIDTH + 200, VIDEO_HEIGHT);

    m_ButtonRecord.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::on_record_clicked));
    m_ButtonPower.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::on_power_clicked));
    m_ButtonQuit.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::on_quit_clicked));
    m_btn_left_fast.set_label("<<");
    m_btn_left.set_label("<");
    m_btn_right.set_label(">");
    m_btn_right_fast.set_label(">>");
    m_btn_left_fast.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::on_left_fast_clicked));
    m_btn_left.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::on_left_clicked));
    m_btn_right.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::on_right_clicked));
    m_btn_right_fast.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::on_right_fast_clicked));
    m_ButtonBox.pack_start(m_ButtonRecord, Gtk::PACK_EXPAND_WIDGET);
    m_ButtonBox.pack_start(m_ButtonPower, Gtk::PACK_EXPAND_WIDGET);
    m_ButtonBox.pack_start(m_ButtonQuit, Gtk::PACK_EXPAND_WIDGET);

    // 2. Setup Video and Drawing Areas
    m_VideoArea.set_size_request(VIDEO_WIDTH, VIDEO_HEIGHT);

    m_DrawingArea.set_size_request(200, 200);

    m_BottomDrawingArea.set_size_request(200, -1);
    m_StatusGrid.set_row_homogeneous(false);
    m_BottomDrawingArea.set_vexpand(true);
    m_BottomDrawingArea.set_valign(Gtk::ALIGN_FILL);
    m_StatusGrid.set_row_spacing(5);
    m_StatusGrid.set_column_spacing(5);
    m_StatusGrid.set_border_width(5);
    m_StatusGrid.attach(m_DrawingArea, 0, 0, 2, 1);
    Gtk::HBox* button_row = Gtk::manage(new Gtk::HBox());
    button_row->set_spacing(5);

    // 2b. Pack all four buttons into the HBox
    button_row->pack_start(m_btn_left_fast, Gtk::PACK_EXPAND_WIDGET); 
    button_row->pack_start(m_btn_left, Gtk::PACK_EXPAND_WIDGET); 
    button_row->pack_start(m_btn_right, Gtk::PACK_EXPAND_WIDGET); 
    button_row->pack_start(m_btn_right_fast, Gtk::PACK_EXPAND_WIDGET);
    m_StatusGrid.attach(*button_row, 0, 1, 2, 1);
    m_StatusGrid.attach(m_BottomDrawingArea, 0, 2, 2, 1);
    m_VideoAndDrawBox.pack_start(m_VideoArea, Gtk::PACK_EXPAND_WIDGET); 
    m_VideoAndDrawBox.pack_start(m_StatusGrid, Gtk::PACK_SHRINK);
    // Also ensure the video area is set to expand vertically (vexpand)
    // This is necessary to fill the entire height of the window.
    m_VideoArea.set_vexpand(true);
    m_VideoArea.set_hexpand(true);
    // 4. Pack the new Horizontal Box and the Button Box into the Main Vertical Box
    m_VBox.pack_start(m_VideoAndDrawBox, Gtk::PACK_EXPAND_WIDGET); // Video/Drawing section expands
    m_VBox.pack_start(m_ButtonBox, Gtk::PACK_SHRINK);              // Button section shrinks

    add(m_VBox);
    show_all_children();
    Glib::signal_timeout().connect(sigc::mem_fun(*this, &MainWindow::on_redraw_timeout), 200);
    char conf_name[PATH_MAX];
    sprintf(conf_name, "%s/%s", getenv("HOME"), CONFIG_FILE);
    printf("Config: %s\n", conf_name);
    load_config(conf_name);
    std::memset(&m_app_config, 0, sizeof(m_app_config));
    get_app_config(&m_app_config);
    setup_pipeline();
    if (m_pipeline) {
        gst_element_set_state(m_pipeline, GST_STATE_PAUSED); 

        // 2. Connect the overlay logic to the realize signal
        m_VideoArea.signal_realize().connect(sigc::mem_fun(*this, &MainWindow::set_video_overlay));

        // 3. Set to PLAYING after initialization
        gst_element_set_state(m_pipeline, GST_STATE_PLAYING);
    }
    control_start(&m_status, &m_app_config);
    crsf_start(&m_status, &m_app_config);
    set_status_line(SEVERITY_NOTIFICATION, "Ready");
}

MainWindow::~MainWindow()
{
    global_window_ptr = nullptr;
    // GStreamer Cleanup
    if (m_pipeline) {
        gst_element_set_state(m_pipeline, GST_STATE_NULL);
        gst_object_unref(m_pipeline);
    }
}

void MainWindow::setup_pipeline()
{
    // Assuming H264, port 5600, payload 96 as per your Python code

    m_pipeline = GST_ELEMENT(gst_pipeline_new("rtp-viewer-pipeline"));

    // --- Element Creation ---
    GstElement *udpsrc = make_gst_element("udpsrc", "udp-source");
    GstElement *rtpdepay = make_gst_element("rtph264depay", "rtp-depay");
    GstElement *parser = make_gst_element("h264parse", "parser");
    GstElement *decoder = make_gst_element("avdec_h264", "decoder");
    m_tee = make_gst_element("tee", "tee");
    GstElement *queue_decoder = make_gst_element("queue", "queue-decoder");
    GstElement *queue_display = make_gst_element("queue", "queue-display");
    GstElement *videoconvert = make_gst_element("videoconvert", "video-convert");

    // autovideosink is flexible and supports GstVideoOverlay on most platforms
//    m_videosink = make_gst_element("autovideosink", "video-sink");
    m_videosink = make_gst_element("xvimagesink", "video-sink");
    // m_videosink = make_gst_element("glimagesink", "video-sink");

    // --- Property Setup ---
    g_object_set(udpsrc, "port", m_app_config.stream_port, NULL);
    GstCaps *caps = gst_caps_from_string("application/x-rtp,payload=96");
    g_object_set(udpsrc, "caps", caps, NULL);
    gst_caps_unref(caps);

    // --- Linking ---
    gst_bin_add_many(GST_BIN(m_pipeline), udpsrc, rtpdepay, parser, m_tee,
            queue_decoder, decoder, videoconvert, queue_display, m_videosink, NULL);

    // Link input to TEE
    if (!gst_element_link_many(udpsrc, rtpdepay, parser, m_tee, NULL)) {
        std::cerr << "Error: Could not link main chain to tee." << std::endl;
        return;
    }

    // Link display branch (get static pad from queue)
    GstPad *tee_src_pad_display = gst_element_request_pad_simple(m_tee, "src_%u");
    GstPad *queue_sink_pad = gst_element_get_static_pad(queue_decoder, "sink");

    if (gst_pad_link(tee_src_pad_display, queue_sink_pad) != GST_PAD_LINK_OK) {
        std::cerr << "Error: Could not link tee to display queue." << std::endl;
        return;
    }
    gst_object_unref(queue_sink_pad);
    gst_object_unref(tee_src_pad_display); // Release request pad reference

    if (!gst_element_link_many(queue_decoder, decoder, videoconvert, queue_display, m_videosink, NULL)) {
        std::cerr << "Error: Could not link display branch elements." << std::endl;
        return;
    }

    // Message Bus setup
    Glib::RefPtr<Gst::Bus> bus = Glib::wrap(gst_pipeline_get_bus(GST_PIPELINE(m_pipeline)), true);
    bus->add_watch(sigc::mem_fun(*this, &MainWindow::on_bus_message));
}

void MainWindow::set_video_overlay()
{
    GstElement *video_sink = m_videosink;
    std::cout << "Setting video overlay..." << std::endl;
    if (video_sink && GST_IS_VIDEO_OVERLAY(video_sink)) {
        // Get the XID/Window ID from the GTK::DrawingArea
        Glib::RefPtr<Gdk::Window> gdk_window = m_VideoArea.get_window();
        if (gdk_window) {
            GdkWindow* c_gdk_window = gdk_window->gobj(); 
        
            // Use the GDK C function to get the XID (guintptr is compatible with XID)
            guintptr window_handle = GDK_WINDOW_XID(c_gdk_window);

            std::cout << "Before gst_video_overlay_set_window_handle" << std::endl;
            gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(video_sink), window_handle);
        }
    }
}

// --- Recording Logic ---
void MainWindow::start_recording()
{
    if (m_status.recording) return;

    // Use a fixed path for simplicity, mirroring the spirit of MOUNT_HELPER_PATH
    std::string mount_point = getenv("HOME");
    mount_point += "/Recordings";

    // Generate filename with timestamp
    std::time_t t = std::time(nullptr);
    char timestamp_buf[30];
    if (std::strftime(timestamp_buf, sizeof(timestamp_buf), "%Y%m%d_%H%M%S", std::localtime(&t)) == 0) {
        std::cerr << "Error: Failed to create timestamp." << std::endl;
        return;
    }
    std::string filename = mount_point + "/recording_" + std::string(timestamp_buf) + ".mp4";

    // 2. Create recording branch elements
    GstElement *queue_record = make_gst_element("queue", "queue-record");
    GstElement *mp4mux = make_gst_element("mp4mux", "muxer");
    GstElement *filesink = make_gst_element("filesink", "file-sink");

    if (!queue_record || !mp4mux || !filesink) return;

    g_object_set(filesink, "location", filename.c_str(), NULL);

    m_record_elements = {queue_record, mp4mux, filesink};

    // 3. Add to pipeline
    for (GstElement *el : m_record_elements) {
        gst_bin_add(GST_BIN(m_pipeline), el);
    }

    // 4. Link recording branch
    m_tee_record_pad = gst_element_request_pad_simple(m_tee, "src_%u");
    GstPad *queue_sink_pad = gst_element_get_static_pad(queue_record, "sink");

    if (gst_pad_link(m_tee_record_pad, queue_sink_pad) != GST_PAD_LINK_OK) {
        std::cerr << "Error: Could not link tee to recording queue." << std::endl;
        // Clean up immediately if linking fails
        for (GstElement *el : m_record_elements) gst_object_unref(el);
        m_record_elements.clear();
        return;
    }
    gst_object_unref(queue_sink_pad);

    if (!gst_element_link_many(queue_record, mp4mux, filesink, NULL)) {
        std::cerr << "Error: Could not link recording elements." << std::endl;
        return;
    }

    // 5. Sync state with parent (pipeline is already PLAYING)
    for (GstElement *el : m_record_elements) {
        gst_element_sync_state_with_parent(el);
    }

    m_status.recording = true;
    std::cout << "Started recording to: " << filename << std::endl;
    m_ButtonRecord.set_label("Stop Recording");
    set_status_line(SEVERITY_WARN, "Recording");
}

void MainWindow::stop_recording()
{
    if (!m_status.recording || m_record_elements.empty()) return;

    // Send EOS to the first element of the recording branch (queue)
    GstPad *pad = gst_element_get_static_pad(m_record_elements.front(), "sink");
    if (pad) {
        gst_pad_send_event(pad, gst_event_new_eos());
        gst_object_unref(pad);
    }

    // Set a timeout to clean up after EOS has time to propagate
    Glib::signal_timeout().connect_once(sigc::mem_fun(*this, &MainWindow::cleanup_recording_branch), 100);
    set_status_line(SEVERITY_NOTIFICATION, "Ready");
}

void MainWindow::cleanup_recording_branch()
{
    if (!m_status.recording)
        return;

    // 1. Unlink and release tee pad
    if (m_tee_record_pad) {
        GstPad *queue_sink_pad = gst_element_get_static_pad(m_record_elements.front(), "sink");

        for (GstElement *el : m_record_elements) {
            gst_element_set_state(el, GST_STATE_NULL);
        }
        if (queue_sink_pad) {
             gst_pad_unlink(m_tee_record_pad, queue_sink_pad);
             gst_object_unref(queue_sink_pad);
        }
        gst_element_release_request_pad(m_tee, m_tee_record_pad);
        gst_object_unref(m_tee_record_pad);
        m_tee_record_pad = nullptr;
    }

    // 2. Set to NULL and remove elements
    for (GstElement *el : m_record_elements) {
        gst_bin_remove(GST_BIN(m_pipeline), el);
        gst_object_unref(el);
    }
    m_record_elements.clear();
    gst_element_set_state(m_pipeline, GST_STATE_READY); 
    m_status.recording = false;
    std::cout << "Recording stopped. Running sync..." << std::endl;
    gst_element_set_state(m_pipeline, GST_STATE_PLAYING);
    // Execute sync as in Python code
    system("sync");
    m_ButtonRecord.set_label("Start Recording");
}

bool MainWindow::on_bus_message(const Glib::RefPtr<Gst::Bus>& bus, const Glib::RefPtr<Gst::Message>& message)
{
    Gst::MessageType type = message->get_message_type();

    if (type == Gst::MESSAGE_ERROR) {
// 1. Get the raw C GstMessage pointer
        GstMessage *c_message = message->gobj();

        GError *gerror = nullptr;
        gchar *debug_info_c = nullptr;

        // 2. Use the C-API function to parse the error
        gst_message_parse_error(c_message, &gerror, &debug_info_c);
        
        // 3. Convert C types to C++ types for logging
        Glib::Error error(gerror); // Wrap GError* into Glib::Error
        Glib::ustring debug_info(debug_info_c);

        std::cerr << "GStreamer Error: " << error.what();
        if (!debug_info.empty()) {
            std::cerr << " (Debug: " << debug_info << ")";
        }
        std::cerr << std::endl;

        // 4. Free the allocated C memory
        g_error_free(gerror);
        g_free(debug_info_c);
        
        // Stop recording if an error occurs
        if (m_status.recording) {
            stop_recording(); 
        }
    } else if (type == Gst::MESSAGE_EOS) {
        // Handle EOS
    }
    return true;
}

void MainWindow::show_help_dialog()
{
    Gtk::MessageDialog dialog( *this, "Довідка про Програму", // Заголовок
        false,             // use_markup (ні)
        Gtk::MESSAGE_INFO, // Тип повідомлення (іконка)
        Gtk::BUTTONS_CLOSE // Кнопки: лише "Закрити" (Close)
    );
    dialog.set_secondary_text( "Довідка.\n"
        "Натисніть 'Закрити', щоб повернутися до основної програми.\n"
        "Кнопки вліво/вправо - повернення антени вліво-вправо приблизно на 1 градус\n"
        "Кнопки вліво/вправо з натиснутим Ctrl - повернення антени вліво-вправо приблизно на 4 градуси\n"
        "F5 - Почати/завершити запис відео\n"
        "F9 - Показати/сховати праву панель в повноекранному режимі\n"
        "F11 - Вхід/вихід повноекранний режим\n"
    );
    dialog.run();
}

bool MainWindow::on_key_press_event(GdkEventKey* event)
{
    switch (event->keyval) {
        case GDK_KEY_f:
        case GDK_KEY_F:
            if (event->state & Gdk::CONTROL_MASK) {
                unsigned int size = VRX_TABLE_COUNT;
                get_table_info(m_status.vrx_table, &size);
                m_status.bands = size;
                crsf_send_vrxtable();
                return true;
            }
        case GDK_KEY_Left:
            m_status.encoder_cnt -= ENCODER_INCREMENT;
            if (event->state & Gdk::CONTROL_MASK)
                SET_BIT(m_status.switch_status, SWITCH_ENCODER_NUM);
            else
                CLEAR_BIT(m_status.switch_status, SWITCH_ENCODER_NUM);
            control_send_message();
            return true;
        case GDK_KEY_Right:
            m_status.encoder_cnt += ENCODER_INCREMENT;
            if (event->state & Gdk::CONTROL_MASK)
                SET_BIT(m_status.switch_status, SWITCH_ENCODER_NUM);
            else
                CLEAR_BIT(m_status.switch_status, SWITCH_ENCODER_NUM);
            control_send_message();
            return true;
        case GDK_KEY_F1:
            show_help_dialog();
            return true;
        case GDK_KEY_F5:
            on_record_clicked();
            return true;
        case GDK_KEY_F9:
            if (!m_StatusGrid.is_visible())
                m_StatusGrid.show();
            else
                m_StatusGrid.hide();
            return true;
        case GDK_KEY_F11: {
            Gdk::WindowState state = get_window()->get_state();
            if (state & Gdk::WINDOW_STATE_FULLSCREEN) {
                unfullscreen(); 
                m_StatusGrid.show();
                m_ButtonBox.show();
            } else {
                fullscreen();
                m_StatusGrid.hide();
                m_ButtonBox.hide();
            }
            return true; 
        }
    }
    
    return Gtk::Window::on_key_press_event(event);
}

// --- Button Handlers ---
void MainWindow::on_record_clicked()
{
    if (!m_status.recording) {
        start_recording();
    } else {
        stop_recording();
    }
}

void MainWindow::on_power_clicked()
{
    if CHECK_BIT(m_status.power_status, SWITCH_ANTENNA_NUM) {
        CLEAR_BIT(m_status.switch_status, SWITCH_ANTENNA_NUM);
    } else {
        SET_BIT(m_status.switch_status, SWITCH_ANTENNA_NUM);
    }
    control_send_message();
}

void MainWindow::on_quit_clicked()
{
    crsf_stop();
    control_stop();
    Gtk::Main::quit();
}

bool MainWindow::on_redraw_timeout()
{
    m_DrawingArea.queue_draw(); 
    m_BottomDrawingArea.queue_draw(); 
    return true; 
}

void MainWindow::on_left_fast_clicked()
{
    SET_BIT(m_status.switch_status, SWITCH_ENCODER_NUM);
    m_status.encoder_cnt -= ENCODER_INCREMENT;
    control_send_message();
}

void MainWindow::on_left_clicked()
{
    CLEAR_BIT(m_status.switch_status, SWITCH_ENCODER_NUM);
    m_status.encoder_cnt -= ENCODER_INCREMENT;
    control_send_message();
}

void MainWindow::on_right_clicked()
{
    CLEAR_BIT(m_status.switch_status, SWITCH_ENCODER_NUM);
    m_status.encoder_cnt += ENCODER_INCREMENT;
    control_send_message();
}

void MainWindow::on_right_fast_clicked()
{
    SET_BIT(m_status.switch_status, SWITCH_ENCODER_NUM);
    m_status.encoder_cnt += ENCODER_INCREMENT;
    control_send_message();
}

void MainWindow::update_status(void)
{
    Glib::signal_timeout().connect(sigc::mem_fun(*this, &MainWindow::on_update_status), 100);
}

void MainWindow::set_status_line(int severity, const char *status)
{
    pthread_mutex_lock(&m_status.mutex);
    if (m_status.status_line)
        free(m_status.status_line);
    if (status)
        m_status.status_line = strdup(status);
    else
        m_status.status_line = NULL;
    m_status.status_severity = severity;

    pthread_mutex_unlock(&m_status.mutex);
}

bool MainWindow::on_update_status(void)
{
    if (CHECK_BIT(m_status.power_status, POWER_BIT)) {
        m_ButtonPower.set_label("POWER ON");
    } else {
        m_ButtonPower.set_label("POWER OFF");
    }
    return false;
}

extern "C" {
void update_status(void)
{
    if (global_window_ptr)
        global_window_ptr->update_status();
}

void set_status_line(int severity, const char *status)
{
    if (global_window_ptr) {
        global_window_ptr->set_status_line(severity, status);
    }
}
}
