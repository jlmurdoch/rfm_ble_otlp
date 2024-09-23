#ifndef __OTELPROTOBUF_H_
#define __OTELPROTOBUF_H_

#include <Arduino.h>
// #include <stdint.h>
#include "time.h"

/*
 * Nanopb protobuf
 */
#include "nanopb/pb.h"
#include "nanopb/pb_encode.h"
#include "nanopb/pb_common.h"

/*
 * Compiled OpenTelemetry Proto headers
 */
#include "nanopb/metrics.pb.h"
#include "nanopb/common.pb.h"
#include "nanopb/resource.pb.h"

// Protobuf setup
#define MAX_PROTOBUF_BYTES 4096

// Attributes struct
typedef struct anode *Attrptr;
typedef struct anode
{
    char *key;
    char *value;
    Attrptr next;
} Attrnode;

// Datapoints
typedef struct dnode *Datapointptr;
typedef struct dnode
{
    uint64_t time;
    int type;
    union
    {
        int64_t as_int;
        double as_double;
    } value;
    Attrptr aHead;
    Attrptr aTail;
    Datapointptr next;
} Datapointnode;

// Metrics
typedef struct mnode *Metricptr;
typedef struct mnode
{
    char *name;
    char *description;
    char *unit;
    int datatype;
    int aggregation;
    bool monotonic;
    Datapointptr dpHead;
    Datapointptr dpTail;
    Metricptr next;
} Metricnode;

// Resource + Scope
typedef struct rnode *Resourceptr;
typedef struct rnode
{
    Attrptr aHead;
    Attrptr aTail;
    Metricptr mHead;
    Metricptr mTail;
    Resourceptr next;
} Resourcenode;

// To enumerate the metric value types
enum
{
    AS_INT,
    AS_DOUBLE
};

enum {
    METRIC_GAUGE,
    METRIC_SUM,
    METRIC_HISTOGRAM,
    METRIC_EXP_HISTOGRAM,
    METRIC_SUMMARY
};

enum {
    AGG_UNSPECIFIED,
    AGG_DELTA,
    AGG_CUMULATIVE
};

uint64_t getEpochNano(void);

// Protobuf encoding of a string
bool encode_string(pb_ostream_t *stream, const pb_field_t *field, void *const *arg);

// Protobuf encoding of Key-Value pairs (Attributes in OpenTelemetry)
bool KeyValue_encode_attributes(pb_ostream_t *ostream, const pb_field_iter_t *field, void *const *arg);

// Protobuf encoding of a Sum datapoint
bool Sum_encode_data_points(pb_ostream_t *ostream, const pb_field_iter_t *field, void *const *arg);

// Protobuf encoding of a Metric definition
bool ScopeMetrics_encode_metric(pb_ostream_t *ostream, const pb_field_iter_t *field, void *const *arg);

// Protobuf encoding of a scope (passthrough - nothing much done here)
bool ResourceMetrics_encode_scope_metrics(pb_ostream_t *ostream, const pb_field_iter_t *field, void *const *arg);

// Protobuf encoding of entire payload
bool MetricsData_encode_resource_metrics(pb_ostream_t *ostream, const pb_field_iter_t *field, void *const *arg);

size_t buildProtobuf(Resourceptr args, uint8_t *pbufPayload, size_t pbufMaxSize);

void addResAttr(Resourceptr p, char *key, char *value);

void addDpAttr(Resourceptr p, char *key, char *value);

void addDatapointDouble(Resourceptr p, double value);

void addDatapoint(Resourceptr p, int type, void *arg);

// Resourceptr p, char *name, char *description, char *unit, int datatype, int aggregation, bool is_monotonic
void addMetric(Resourceptr p, char *name, char *description, char *unit, int datatype, int aggregation, bool is_monotonic);

Resourceptr addOteldata(void);

void freeOteldata(Resourceptr p);

void printOteldata(Resourceptr p);

#endif
