#ifndef LIBRARIAN_PANEL_H
#define LIBRARIAN_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <rviz/config.h>
#include <QtWidgets>
#include <librarian_resources/Book.h>
#include <librarian_resources/Books.h>
#include <librarian_resources/Environment.h>
#include <cv_bridge/cv_bridge.h>
#include <string>

namespace librarian_panel
{
    /**
     * @brief Shows the perceived environment.
     * 
     * The LibrarianPanel is an rviz panel that displays the contents of the
     * Environment message emitted by the vision pipeline.
     */
    class LibrarianPanel : public rviz::Panel
    {
        Q_OBJECT
    public:
        explicit LibrarianPanel(QWidget *parent = nullptr);
        ~LibrarianPanel() override;

    protected:
        ros::NodeHandle nh_;
        ros::Subscriber books_sub;
        QTableWidget *table;
        void receive_books(const librarian_resources::Environment::ConstPtr &msg);
    };
}

#endif

