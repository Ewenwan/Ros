/****************************************************************************
** Meta object code from reading C++ file 'CameraTcpServer.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../src/find_object_2d/src/CameraTcpServer.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'CameraTcpServer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_find_object__CameraTcpServer[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      30,   29,   29,   29, 0x08,
      61,   49,   29,   29, 0x08,
     104,   29,   29,   29, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_find_object__CameraTcpServer[] = {
    "find_object::CameraTcpServer\0\0"
    "readReceivedData()\0socketError\0"
    "displayError(QAbstractSocket::SocketError)\0"
    "connectionLost()\0"
};

void find_object::CameraTcpServer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        CameraTcpServer *_t = static_cast<CameraTcpServer *>(_o);
        switch (_id) {
        case 0: _t->readReceivedData(); break;
        case 1: _t->displayError((*reinterpret_cast< QAbstractSocket::SocketError(*)>(_a[1]))); break;
        case 2: _t->connectionLost(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData find_object::CameraTcpServer::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject find_object::CameraTcpServer::staticMetaObject = {
    { &QTcpServer::staticMetaObject, qt_meta_stringdata_find_object__CameraTcpServer,
      qt_meta_data_find_object__CameraTcpServer, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &find_object::CameraTcpServer::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *find_object::CameraTcpServer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *find_object::CameraTcpServer::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_find_object__CameraTcpServer))
        return static_cast<void*>(const_cast< CameraTcpServer*>(this));
    return QTcpServer::qt_metacast(_clname);
}

int find_object::CameraTcpServer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QTcpServer::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
