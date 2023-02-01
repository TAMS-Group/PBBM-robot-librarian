#include "librarian_panel.h"

namespace librarian_panel
{
    LibrarianPanel::LibrarianPanel(QWidget *parent) : rviz::Panel(parent)
    {
        ros::NodeHandle nh, pnh("~");
        books_sub = nh.subscribe("/librarian/environment", 1, &LibrarianPanel::receive_books, this);

        auto *main_layout = new QVBoxLayout;
        main_layout->setAlignment(Qt::AlignTop);

        table = new QTableWidget(0, 4);
        table->setHorizontalHeaderItem(0, new QTableWidgetItem("ID"));
        table->setHorizontalHeaderItem(1, new QTableWidgetItem("Confidence"));
        table->setHorizontalHeaderItem(2, new QTableWidgetItem("Size"));
        table->setHorizontalHeaderItem(3, new QTableWidgetItem("Text"));
        table->horizontalHeader()->setStretchLastSection(true);
        table->verticalHeader()->hide();
        table->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
        main_layout->addWidget(table);

        setLayout(main_layout);
    }

    LibrarianPanel::~LibrarianPanel() {}

    void LibrarianPanel::receive_books(const librarian_resources::Environment::ConstPtr &msg)
    {
        // Clear the table
        table->clearContents();
        table->setRowCount(0);
        
        for (librarian_resources::Book book : msg->books)
        {
            table->insertRow(0);
            table->setItem(0, 0, new QTableWidgetItem(QString::fromStdString(std::to_string(book.database_id))));
            table->setItem(0, 1, new QTableWidgetItem(QString::fromStdString(std::to_string(book.confidence))));
            
            // Display units in millimetres rather than metres
            std::string width = std::to_string((int)(book.recognition.spine_size[0] * 1000));
            std::string height = std::to_string((int)(book.recognition.spine_size[1] * 1000));
            table->setItem(0, 2, new QTableWidgetItem(QString::fromStdString(width + "mm x " + height + "mm")));

            // Try to display the image of the spine (though the icon is really small)
            cv_bridge::CvImagePtr spine = cv_bridge::toCvCopy(book.recognition.spine);
            cv::Mat cvim = spine->image;
            cv::rotate(cvim, cvim, cv::ROTATE_90_CLOCKWISE);
            cv::cvtColor(cvim, cvim, cv::COLOR_BGR2RGB);
            QImage img((uchar *)cvim.data, cvim.cols, cvim.rows, QImage::Format_RGB888);
            QIcon icon = QIcon(QPixmap::fromImage(img));

            table->setItem(0, 3, new QTableWidgetItem(icon, QString::fromStdString(book.title.data)));
        }
    }
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(librarian_panel::LibrarianPanel, rviz::Panel)
