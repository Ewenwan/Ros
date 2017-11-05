/****************************************************************************
** Meta object code from reading C++ file 'TcpServer.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../../../src/src/find_object_2d/include/find_object/TcpServer.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'TcpServer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_find_object__TcpServer[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      27,   24,   23,   23, 0x05,
      58,   23,   23,   23, 0x05,
      76,   23,   23,   23, 0x05,

 // slots: signature, parameters, type, tag, flags
     103,   98,   23,   23, 0x0a,
     152,   23,   23,   23, 0x08,
     164,   23,   23,   23, 0x08,
     195,  183,   23,   23, 0x08,
     238,   23,   23,   23, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_find_object__TcpServer[] = {
    "find_object::TcpServer\0\0,,\0"
    "addObject(cv::Mat,int,QString)\0"
    "removeObject(int)\0detectObject(cv::Mat)\0"
    "info\0publishDetectionInfo(find_object::DetectionInfo)\0"
    "addClient()\0readReceivedData()\0"
    "socketError\0displayError(QAbstractSocket::SocketError)\0"
    "connectionLost()\0"
};

void find_object::TcpServer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        TcpServer *_t = static_cast<TcpServer *>(_o);
        switch (_id) {
        case 0: _t->addObject((*reinterpret_cast< const cv::Mat(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< const QString(*)>(_a[3]))); break;
        case 1: _t->removeObject((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->detectObject((*reinterpret_cast< const cv::Mat(*)>(_a[1]))); break;
        case 3: _t->publishDetectionInfo((*reinterpret_cast< const find_object::DetectionInfo(*)>(_a[1]))); break;
        case 4: _t->addClient(); break;
        case 5: _t->readReceivedData(); break;
        case 6: _t->displayError((*reinterpret_cast< QAbstractSocket::SocketError(*)>(_a[1]))); break;
        case 7: _t->connectionLost(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData find_object::TcpServer::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject find_object::TcpServer::staticMetaObject = {
    { &QTcpServer::staticMetaObject, qt_meta_stringdata_find_object__TcpServer,
      qt_meta_data_find_object__TcpServer, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &find_object::TcpServer::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *find_object::TcpServer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *find_object::TcpServer::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_find_object__TcpServer))
        return static_cast<void*>(const_cast< TcpServer*>(this));
    return QTcpServer::qt_metacast(_clname);
}

int find_object::TcpServer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QTcpServer::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
void find_object::TcpServer::addObject(const cv::Mat & _t1, int _t2, const QString & _t3)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void find_object::TcpServer::removeObject(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void find_object::TcpServer::detectObject(const cv::Mat & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_END_MOC_NAMESPACE
