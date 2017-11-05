/****************************************************************************
** Meta object code from reading C++ file 'ObjWidget.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../../src/find_object_2d/include/find_object/ObjWidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ObjWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_find_object__ObjWidget[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      24,   23,   23,   23, 0x05,
      66,   23,   23,   23, 0x05,
      85,   23,   23,   23, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_find_object__ObjWidget[] = {
    "find_object::ObjWidget\0\0"
    "removalTriggered(find_object::ObjWidget*)\0"
    "selectionChanged()\0roiChanged(cv::Rect)\0"
};

void find_object::ObjWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ObjWidget *_t = static_cast<ObjWidget *>(_o);
        switch (_id) {
        case 0: _t->removalTriggered((*reinterpret_cast< find_object::ObjWidget*(*)>(_a[1]))); break;
        case 1: _t->selectionChanged(); break;
        case 2: _t->roiChanged((*reinterpret_cast< const cv::Rect(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData find_object::ObjWidget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject find_object::ObjWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_find_object__ObjWidget,
      qt_meta_data_find_object__ObjWidget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &find_object::ObjWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *find_object::ObjWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *find_object::ObjWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_find_object__ObjWidget))
        return static_cast<void*>(const_cast< ObjWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int find_object::ObjWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void find_object::ObjWidget::removalTriggered(find_object::ObjWidget * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void find_object::ObjWidget::selectionChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void find_object::ObjWidget::roiChanged(const cv::Rect & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_END_MOC_NAMESPACE
