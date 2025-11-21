#pragma once

#include <gtkmm.h>
#include <cairomm/context.h>
#include <cairomm/surface.h>
#include "control.h"

class DrawArea : public Gtk::DrawingArea {
public:
    DrawArea() = default;
    void set_status_ptr(global_status_t *status_ptr) { m_status = status_ptr; } 
protected:
    Cairo::RefPtr<Cairo::ImageSurface> m_buffer_surface;
    void on_size_allocate(Gtk::Allocation& allocation) override;
    void draw_text(Cairo::RefPtr<Cairo::Context>& cr, const char *text, double &y, int inverted);
    global_status_t *m_status;
};

class AntennaDrawArea : public DrawArea {
public:
    AntennaDrawArea();
protected:
    bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) override;
};

class VRXDrawArea : public DrawArea {
public:
    VRXDrawArea();
protected:
    bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) override;
};
