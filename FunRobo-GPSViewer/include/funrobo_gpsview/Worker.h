#ifndef __WORKER_H__
#define __WORKER_H__

#include <QObject>

class Worker : public QObject {
    Q_OBJECT
public:
    Worker();
    virtual ~Worker();
public slots:
    void process();
signals:
    void finished();
};

#endif
