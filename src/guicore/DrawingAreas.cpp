#include "DrawingAreas.h"
#include <cstring>
#include <cmath>
#include "utils.h"
#include "common.h"
#include "protocol.h"
#include "config.h"

#define RADIUS 50
#define MARK_LEN 10
//#define X_OFFSET 20.0
#define Y_OFFSET 100.0

void DrawArea::on_size_allocate(Gtk::Allocation& allocation)
{
    // Call the base class method
    Gtk::DrawingArea::on_size_allocate(allocation);

    const int width = allocation.get_width();
    const int height = allocation.get_height();

    // Create or recreate the buffer surface if the size changes
    if (!m_buffer_surface ||
        m_buffer_surface->get_width() != width ||
        m_buffer_surface->get_height() != height)
    {
        // Use Cairo::FORMAT_ARGB32 for a high-quality buffer with alpha support
        m_buffer_surface = Cairo::ImageSurface::create(Cairo::FORMAT_ARGB32, width, height);
    }
}

AntennaDrawArea::AntennaDrawArea()
{
//    set_size_request(200, 200);
//    m_angle = 0;
}

#define PADDING_H 5.0
#define PADDING_V 3.0
void DrawArea::draw_text(Cairo::RefPtr<Cairo::Context>& cr, const char *text, double &y, int inverted)
{
    Cairo::TextExtents te;

    cr->get_text_extents(text, te);
    y += te.height + 10;
    if (inverted) {
        double rect_width = te.width + 2 * PADDING_H;
        double rect_height = te.height + 2 * PADDING_V;

        cr->set_source_rgb(0.8, 0.8, 0.8);
        cr->rectangle(20 - te.x_bearing - PADDING_H, y - PADDING_V, rect_width, rect_height);
        cr->fill();
        cr->set_source_rgb(0, 0, 0);
    } else {
        cr->set_source_rgb(0.8, 0.8, 0.8);
    }
    cr->move_to(20 - te.x_bearing, y - te.y_bearing);
    cr->show_text(text);
}

bool AntennaDrawArea::on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
{
    Gtk::Allocation allocation = get_allocation();
    const int width = allocation.get_width();
    const int height = allocation.get_height();
    static uint64_t timestamp = 0;
    double circle_radius;

    if (!m_buffer_surface) {
        // Should not happen if on_size_allocate is implemented correctly
        return false; 
    }
    Cairo::RefPtr<Cairo::Context> buffer_context = Cairo::Context::create(m_buffer_surface);
    buffer_context->set_source_rgb(0.2, 0.2, 0.2);
    buffer_context->rectangle(0, 0, width, height);
    buffer_context->fill();

    double rad;
    double sn, cs;
    double x, y;
    char buf[100];
    int temp_angle = m_status->angle;
    double x_offset;

    buffer_context->select_font_face("Sans", Cairo::FONT_SLANT_NORMAL, Cairo::FONT_WEIGHT_NORMAL);
    buffer_context->set_font_size(14.0);
    buffer_context->set_line_width(1);
    buffer_context->set_source_rgb(1, 1, 1);
    x_offset = width / 2 - RADIUS;

    for (int angle = -120; angle <= 120; angle += 10) {
        rad = (double)angle * M_PI / 180.0;
        sn = sin(rad);
        cs = cos(rad);
        x = x_offset + RADIUS + RADIUS * sn;
        y = Y_OFFSET - RADIUS * cs;
        buffer_context->move_to(x, y);
        x = x_offset + RADIUS + (RADIUS + MARK_LEN) * sn;
        y = Y_OFFSET - (RADIUS + MARK_LEN) * cs;
        buffer_context->line_to(x, y);
    }
    if (m_status->updated) {
        rad = (double)temp_angle * M_PI / 180.0;
        sn = sin(rad);
        cs = cos(rad);
        buffer_context->set_line_width(3);
        x = x_offset + RADIUS + 5 * sn;
        y = Y_OFFSET - 5 * cs;
        buffer_context->move_to(x, y);
        x = x_offset + RADIUS + (RADIUS + MARK_LEN + 3) * sn;
        y = Y_OFFSET - (RADIUS + MARK_LEN + 3) * cs;
        buffer_context->line_to(x, y);
        sprintf(buf, "%d", temp_angle);
        x = x_offset + RADIUS + RADIUS * sin(-120.0 * M_PI / 180);
        double x1 = x_offset + RADIUS + RADIUS * sin(120.0 * M_PI / 180);
        buffer_context->set_source_rgb(1, 1, 1);
        Cairo::TextExtents te;
        buffer_context->get_text_extents(buf, te);
        x = x + (x1 - x - te.width) / 2;
        y = Y_OFFSET - RADIUS * cos(120.0 * M_PI / 180);
        buffer_context->move_to(x - te.x_bearing, y - te.y_bearing);
        buffer_context->show_text(buf);
    }
    buffer_context->stroke();
    if (m_status->recording) {
        uint64_t now = get_timestamp();
        if (!timestamp)
            timestamp = now;
        if (now - timestamp <= 500) {
            circle_radius = 8.0;
            buffer_context->set_source_rgb(1.0, 0.0, 0.0);
            buffer_context->arc(circle_radius+5, circle_radius+5, circle_radius, 0, 2 * M_PI);
            buffer_context->fill();
        } else if (now - timestamp >= 1000) {
            timestamp = 0;
        }
    } else {
        timestamp = 0;
    }
    cr->set_source(m_buffer_surface, 0, 0);
    cr->paint();
    return true;
}

VRXDrawArea::VRXDrawArea()
{
//    set_size_request(200, -1);
}

bool VRXDrawArea::on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
{
    Gtk::Allocation allocation = get_allocation();
    const int width = allocation.get_width();
    const int height = allocation.get_height();
    char buf[100];
    int temp_power, temp_lp;
    Cairo::TextExtents te;
    double y;
    static uint64_t last_flag_timestamp;

    Cairo::RefPtr<Cairo::Context> buffer_context = Cairo::Context::create(m_buffer_surface);
    buffer_context->set_source_rgb(0.2, 0.2, 0.2);
    buffer_context->rectangle(0, 0, width, height);
    buffer_context->fill();
    y = 10;
    temp_power = CHECK_BIT(m_status->power_status, POWER_BIT);
    temp_lp = CHECK_BIT(m_status->power_status, LOW_POWER_BIT);
    sprintf(buf, "%s",
    m_status->updated ? (temp_power ? "Powered" : "POWER OFF") : "Status UNKNOWN");
    if (temp_power || !m_status->updated)
        buffer_context->set_source_rgb(1, 1, 1);
    else
        buffer_context->set_source_rgb(0.9, 0, 0);
    buffer_context->set_font_size(12);
    buffer_context->get_text_extents(buf, te);
    buffer_context->move_to(20 - te.x_bearing, y - te.y_bearing);
    buffer_context->show_text(buf);
    buffer_context->set_source_rgb(0.8, 0.8, 0.8);
    sprintf(buf, "%ld of %ld", m_status->rc_packets_good, m_status->rc_packets);
    buffer_context->move_to(10, height - 50);
    buffer_context->show_text(buf);
    sprintf(buf, "%ld/errs %ld", m_status->rc_bytes, m_status->rc_errs);
    buffer_context->move_to(10, height - 30);
    buffer_context->show_text(buf);
    if (m_status->updated) {
        if (m_status->recording) {
            buffer_context->set_source_rgb(0.8, 0.1, 0.1);
            buffer_context->move_to(10, height - 10);
            buffer_context->show_text("RECORDING...");
        } else {
            buffer_context->set_source_rgb(0.1, 0.5, 0.1); // Green
            buffer_context->move_to(10, height - 10);
            buffer_context->show_text("Ready");
        }
    }
    y += te.height + 10;                                                                                                                                                                            
    if (m_status->connect_status && m_status->updated && temp_lp) {
        strcpy(buf, "LOW POWER");
        buffer_context->set_source_rgb(0.9, 0, 0);
        buffer_context->get_text_extents(buf, te);
        buffer_context->move_to(20 - te.x_bearing, y - te.y_bearing);
        buffer_context->show_text(buf);
        y += te.height + 10;
    }
    if (!m_status->connect_status) {
        buffer_context->set_source_rgb(0.9, 0, 0);
        sprintf(buf, "NO CONNECTION");
        buffer_context->get_text_extents(buf, te);
        buffer_context->move_to(20 - te.x_bearing, y - te.y_bearing);
        buffer_context->show_text(buf);
    }
    if (temp_power) {
        struct channel_data data[CHANNELS_CNT];
        unsigned int size = CHANNELS_CNT;
        int show_vrx;

        show_vrx = 0;
        if (m_status->channels_updated) {
            m_status->channels_updated = 0;
            last_flag_timestamp = get_timestamp();
            show_vrx = 1;
        } else {
            if (last_flag_timestamp && (get_timestamp() - last_flag_timestamp <= 3000))
                show_vrx = 1;
        }
        if (show_vrx) {
            if (get_chan_info(&m_status->channels, data, &size)) {
                buffer_context->set_font_size(16);
                y += 10;
                for (unsigned int i = 0; i < size; i++) {
                    sprintf(buf, "%s:%d", data[i].band_name, data[i].freq);
                    draw_text(buffer_context, buf, y, data[i].selected);
                }
                y += 10;
                strcpy(buf, "TX1");
                draw_text(buffer_context, "TX1", y,
                          (!data->tx_selected || data->tx_selected == 1));
                draw_text(buffer_context, "TX2", y,
                          (data->tx_selected == 1 || data->tx_selected == 2));
            }
        }
    }

    cr->set_source(m_buffer_surface, 0, 0);
    cr->paint();
    return true;
}
