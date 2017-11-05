/****************************************************************************
** Meta object code from reading C++ file 'ParametersToolBox.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../src/find_object_2d/src/ParametersToolBox.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ParametersToolBox.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_find_object__ParametersToolBox[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      37,   32,   31,   31, 0x05,

 // slots: signature, parameters, type, tag, flags
      68,   31,   31,   31, 0x08,
      92,   86,   31,   31, 0x08,
     117,   86,   31,   31, 0x08,
     139,   86,   31,   31, 0x08,
     160,   31,   31,   31, 0x08,
     179,   31,   31,   31, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_find_object__ParametersToolBox[] = {
    "find_object::ParametersToolBox\0\0name\0"
    "parametersChanged(QStringList)\0"
    "changeParameter()\0value\0"
    "changeParameter(QString)\0changeParameter(bool)\0"
    "changeParameter(int)\0resetCurrentPage()\0"
    "resetAllPages()\0"
};

void find_object::ParametersToolBox::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ParametersToolBox *_t = static_cast<ParametersToolBox *>(_o);
        switch (_id) {
        case 0: _t->parametersChanged((*reinterpret_cast< const QStringList(*)>(_a[1]))); break;
        case 1: _t->changeParameter(); break;
        case 2: _t->changeParameter((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 3: _t->changeParameter((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->changeParameter((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->resetCurrentPage(); break;
        case 6: _t->resetAllPages(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData find_object::ParametersToolBox::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject find_object::ParametersToolBox::staticMetaObject = {
    { &QToolBox::staticMetaObject, qt_meta_stringdata_find_object__ParametersToolBox,
      qt_meta_data_find_object__ParametersToolBox, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &find_object::ParametersToolBox::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *find_object::ParametersToolBox::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *find_object::ParametersToolBox::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_find_object__ParametersToolBox))
        return static_cast<void*>(const_cast< ParametersToolBox*>(this));
    return QToolBox::qt_metacast(_clname);
}

int find_object::ParametersToolBox::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QToolBox::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void find_object::ParametersToolBox::parametersChanged(const QStringList & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
