/****************************************************************************
** Meta object code from reading C++ file 'Ros2ConnectionListModel.hpp'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../src/qml/models/Ros2ConnectionListModel.hpp"
#include <QtCore/qmetatype.h>

#if __has_include(<QtCore/qtmochelpers.h>)
#include <QtCore/qtmochelpers.h>
#else
QT_BEGIN_MOC_NAMESPACE
#endif


#include <memory>

#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Ros2ConnectionListModel.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
QT_WARNING_DISABLE_GCC("-Wuseless-cast")
namespace {

#ifdef QT_MOC_HAS_STRINGDATA
struct qt_meta_stringdata_CLASSros2monitorSCOPERos2ConnectionListModelENDCLASS_t {};
static constexpr auto qt_meta_stringdata_CLASSros2monitorSCOPERos2ConnectionListModelENDCLASS = QtMocHelpers::stringData(
    "ros2monitor::Ros2ConnectionListModel",
    "getRow",
    "",
    "i",
    "role_name"
);
#else  // !QT_MOC_HAS_STRING_DATA
struct qt_meta_stringdata_CLASSros2monitorSCOPERos2ConnectionListModelENDCLASS_t {
    uint offsetsAndSizes[10];
    char stringdata0[37];
    char stringdata1[7];
    char stringdata2[1];
    char stringdata3[2];
    char stringdata4[10];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_CLASSros2monitorSCOPERos2ConnectionListModelENDCLASS_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_CLASSros2monitorSCOPERos2ConnectionListModelENDCLASS_t qt_meta_stringdata_CLASSros2monitorSCOPERos2ConnectionListModelENDCLASS = {
    {
        QT_MOC_LITERAL(0, 36),  // "ros2monitor::Ros2ConnectionLi..."
        QT_MOC_LITERAL(37, 6),  // "getRow"
        QT_MOC_LITERAL(44, 0),  // ""
        QT_MOC_LITERAL(45, 1),  // "i"
        QT_MOC_LITERAL(47, 9)   // "role_name"
    },
    "ros2monitor::Ros2ConnectionListModel",
    "getRow",
    "",
    "i",
    "role_name"
};
#undef QT_MOC_LITERAL
#endif // !QT_MOC_HAS_STRING_DATA
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_CLASSros2monitorSCOPERos2ConnectionListModelENDCLASS[] = {

 // content:
      11,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    2,   20,    2, 0x0a,    1 /* Public */,

 // slots: parameters
    QMetaType::QVariant, QMetaType::Int, QMetaType::QString,    3,    4,

       0        // eod
};

Q_CONSTINIT const QMetaObject ros2monitor::Ros2ConnectionListModel::staticMetaObject = { {
    QMetaObject::SuperData::link<QAbstractListModel::staticMetaObject>(),
    qt_meta_stringdata_CLASSros2monitorSCOPERos2ConnectionListModelENDCLASS.offsetsAndSizes,
    qt_meta_data_CLASSros2monitorSCOPERos2ConnectionListModelENDCLASS,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_CLASSros2monitorSCOPERos2ConnectionListModelENDCLASS_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<Ros2ConnectionListModel, std::true_type>,
        // method 'getRow'
        QtPrivate::TypeAndForceComplete<QVariant, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>
    >,
    nullptr
} };

void ros2monitor::Ros2ConnectionListModel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Ros2ConnectionListModel *>(_o);
        (void)_t;
        switch (_id) {
        case 0: { QVariant _r = _t->getRow((*reinterpret_cast< std::add_pointer_t<int>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2])));
            if (_a[0]) *reinterpret_cast< QVariant*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    }
}

const QMetaObject *ros2monitor::Ros2ConnectionListModel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ros2monitor::Ros2ConnectionListModel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CLASSros2monitorSCOPERos2ConnectionListModelENDCLASS.stringdata0))
        return static_cast<void*>(this);
    return QAbstractListModel::qt_metacast(_clname);
}

int ros2monitor::Ros2ConnectionListModel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QAbstractListModel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 1;
    }
    return _id;
}
QT_WARNING_POP
