/********************************************************************************
** Form generated from reading UI file 'aboutDialog.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ABOUTDIALOG_H
#define UI_ABOUTDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QSpacerItem>
#include <QtGui/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_aboutDialog
{
public:
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label_7;
    QFrame *frame;
    QVBoxLayout *verticalLayout_4;
    QLabel *label;
    QLabel *label_9;
    QGridLayout *gridLayout;
    QLabel *label_2;
    QLabel *label_4;
    QLabel *label_6;
    QLabel *label_HomePage_2;
    QLabel *label_8;
    QLabel *label_version;
    QLabel *label_version_opencv;
    QLabel *label_10;
    QLabel *label_11;
    QLabel *label_version_qt;
    QSpacerItem *verticalSpacer_3;
    QLabel *label_3;
    QLabel *label_5;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *aboutDialog)
    {
        if (aboutDialog->objectName().isEmpty())
            aboutDialog->setObjectName(QString::fromUtf8("aboutDialog"));
        aboutDialog->resize(527, 289);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(aboutDialog->sizePolicy().hasHeightForWidth());
        aboutDialog->setSizePolicy(sizePolicy);
        verticalLayout = new QVBoxLayout(aboutDialog);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label_7 = new QLabel(aboutDialog);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setMaximumSize(QSize(100, 100));
        label_7->setPixmap(QPixmap(QString::fromUtf8(":/images/resources/Find-Object.png")));
        label_7->setScaledContents(true);

        horizontalLayout->addWidget(label_7);

        frame = new QFrame(aboutDialog);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setFrameShape(QFrame::Box);
        frame->setFrameShadow(QFrame::Raised);
        verticalLayout_4 = new QVBoxLayout(frame);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        label = new QLabel(frame);
        label->setObjectName(QString::fromUtf8("label"));
        label->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        verticalLayout_4->addWidget(label);

        label_9 = new QLabel(frame);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_9->setWordWrap(true);

        verticalLayout_4->addWidget(label_9);

        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_2 = new QLabel(frame);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 1, 0, 1, 1);

        label_4 = new QLabel(frame);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout->addWidget(label_4, 3, 0, 1, 1);

        label_6 = new QLabel(frame);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout->addWidget(label_6, 6, 0, 1, 1);

        label_HomePage_2 = new QLabel(frame);
        label_HomePage_2->setObjectName(QString::fromUtf8("label_HomePage_2"));
        label_HomePage_2->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_HomePage_2->setOpenExternalLinks(true);

        gridLayout->addWidget(label_HomePage_2, 3, 1, 1, 1);

        label_8 = new QLabel(frame);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        gridLayout->addWidget(label_8, 1, 1, 1, 1);

        label_version = new QLabel(frame);
        label_version->setObjectName(QString::fromUtf8("label_version"));
        label_version->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        gridLayout->addWidget(label_version, 6, 1, 1, 1);

        label_version_opencv = new QLabel(frame);
        label_version_opencv->setObjectName(QString::fromUtf8("label_version_opencv"));
        label_version_opencv->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        gridLayout->addWidget(label_version_opencv, 8, 1, 1, 1);

        label_10 = new QLabel(frame);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        gridLayout->addWidget(label_10, 8, 0, 1, 1);

        label_11 = new QLabel(frame);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        gridLayout->addWidget(label_11, 7, 0, 1, 1);

        label_version_qt = new QLabel(frame);
        label_version_qt->setObjectName(QString::fromUtf8("label_version_qt"));

        gridLayout->addWidget(label_version_qt, 7, 1, 1, 1);


        verticalLayout_4->addLayout(gridLayout);

        verticalSpacer_3 = new QSpacerItem(20, 1, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_4->addItem(verticalSpacer_3);

        label_3 = new QLabel(frame);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setPixmap(QPixmap(QString::fromUtf8(":/images/IntRoLabSmall.png")));
        label_3->setAlignment(Qt::AlignCenter);

        verticalLayout_4->addWidget(label_3);

        label_5 = new QLabel(frame);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setAlignment(Qt::AlignCenter);

        verticalLayout_4->addWidget(label_5);


        horizontalLayout->addWidget(frame);


        verticalLayout->addLayout(horizontalLayout);

        buttonBox = new QDialogButtonBox(aboutDialog);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Close);

        verticalLayout->addWidget(buttonBox);


        retranslateUi(aboutDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), aboutDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), aboutDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(aboutDialog);
    } // setupUi

    void retranslateUi(QDialog *aboutDialog)
    {
        aboutDialog->setWindowTitle(QApplication::translate("aboutDialog", "About Find-Object", 0, QApplication::UnicodeUTF8));
        label_7->setText(QString());
        label->setText(QApplication::translate("aboutDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Lucida Grande'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'MS Shell Dlg 2'; font-size:14pt; font-weight:600;\">Find-Object</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        label_9->setText(QString());
        label_2->setText(QApplication::translate("aboutDialog", "Author :", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("aboutDialog", "Link :", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("aboutDialog", "Version :", 0, QApplication::UnicodeUTF8));
        label_HomePage_2->setText(QApplication::translate("aboutDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Lucida Grande'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><a href=\"http://code.google.com/p/find-object\"><span style=\" text-decoration: underline; color:#0000ff;\">Home page</span></a></p></body></html>", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("aboutDialog", "Mathieu Labb\303\251, matlabbe@gmail.com", 0, QApplication::UnicodeUTF8));
        label_version->setText(QString());
        label_version_opencv->setText(QString());
        label_10->setText(QApplication::translate("aboutDialog", "OpenCV version :", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("aboutDialog", "Qt version :", 0, QApplication::UnicodeUTF8));
        label_version_qt->setText(QString());
        label_3->setText(QString());
        label_5->setText(QApplication::translate("aboutDialog", "Copyright (C) 2011-2015 IntRoLab - Universit\303\251 de Sherbrooke", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class aboutDialog: public Ui_aboutDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ABOUTDIALOG_H
