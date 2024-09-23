#include "otel-protobuf.h"

// Protobuf encoding of a string
bool encode_string(pb_ostream_t *stream, const pb_field_t *field, void *const *arg)
{
    const char *str = (const char *)(*arg);

    if (!pb_encode_tag_for_field(stream, field))
        return false;

    return pb_encode_string(stream, (const uint8_t *)str, strlen(str));
}

// Protobuf encoding of Key-Value pairs (Attributes in OpenTelemetry)
bool KeyValue_encode_attributes(pb_ostream_t *ostream, const pb_field_iter_t *field, void *const *arg)
{
    Attrptr attrptr = (Attrnode *)(*arg);

    while (attrptr != NULL)
    {
        KeyValue keyvalue = {};

        keyvalue.key.arg = attrptr->key;
        keyvalue.key.funcs.encode = encode_string;

        keyvalue.has_value = true;
        keyvalue.value.which_value = AnyValue_string_value_tag;
        keyvalue.value.value.string_value.arg = attrptr->value;
        keyvalue.value.value.string_value.funcs.encode = encode_string;

        // Build the submessage tag
        if (!pb_encode_tag_for_field(ostream, field))
        {
            const char *error = PB_GET_ERROR(ostream);
            return false;
        }

        // Build the submessage payload
        if (!pb_encode_submessage(ostream, KeyValue_fields, &keyvalue))
        {
            const char *error = PB_GET_ERROR(ostream);
            return false;
        }
        attrptr = attrptr->next;
    }

    return true;
}

bool Gauge_encode_data_points(pb_ostream_t *ostream, const pb_field_iter_t *field, void *const *arg)
{
    Datapointptr datapointptr = (Datapointptr)(*arg);

    while (datapointptr != NULL)
    {
        NumberDataPoint data_point = {};

        data_point.time_unix_nano = datapointptr->time;

        // Two 64-bit number types in OpenTelemetry: Integers or Doubles
        if (datapointptr->type == AS_INT)
        {
            data_point.which_value = NumberDataPoint_as_int_tag;
            data_point.value.as_int = datapointptr->value.as_int;
        }
        else if (datapointptr->type == AS_DOUBLE)
        {
            data_point.which_value = NumberDataPoint_as_double_tag;
            data_point.value.as_double = datapointptr->value.as_double;
        }

        // Do we have attributes to assign to this datapoint?
        if (datapointptr->aHead != NULL)
        {
            data_point.attributes.arg = datapointptr->aHead;
            data_point.attributes.funcs.encode = KeyValue_encode_attributes;
        }

        // Any flags for this?
        data_point.flags = 0;

        // Build the submessage tag
        if (!pb_encode_tag_for_field(ostream, field))
        {
            const char *error = PB_GET_ERROR(ostream);
            return false;
        }

        // Build the submessage payload
        if (!pb_encode_submessage(ostream, NumberDataPoint_fields, &data_point))
        {
            const char *error = PB_GET_ERROR(ostream);
            return false;
        }
        datapointptr = datapointptr->next;
    }

    return true;
}


// Protobuf encoding of a Sum datapoint
bool Sum_encode_data_points(pb_ostream_t *ostream, const pb_field_iter_t *field, void *const *arg)
{
    Datapointptr datapointptr = (Datapointptr)(*arg);

    // if (myDataPoint != NULL)
    while (datapointptr != NULL)
    {
        NumberDataPoint data_point = {};

        data_point.time_unix_nano = datapointptr->time;

        // Two 64-bit number types in OpenTelemetry: Integers or Doubles
        if (datapointptr->type == AS_INT)
        {
            data_point.which_value = NumberDataPoint_as_int_tag;
            data_point.value.as_int = datapointptr->value.as_int;
        }
        else if (datapointptr->type == AS_DOUBLE)
        {
            data_point.which_value = NumberDataPoint_as_double_tag;
            data_point.value.as_double = datapointptr->value.as_double;
        }

        // Do we have attributes to assign to this datapoint?
        if (datapointptr->aHead != NULL)
        {
            data_point.attributes.arg = datapointptr->aHead;
            data_point.attributes.funcs.encode = KeyValue_encode_attributes;
        }

        // Any flags for this?
        data_point.flags = 0;

        // Build the submessage tag
        if (!pb_encode_tag_for_field(ostream, field))
        {
            const char *error = PB_GET_ERROR(ostream);
            return false;
        }

        // Build the submessage payload
        if (!pb_encode_submessage(ostream, NumberDataPoint_fields, &data_point))
        {
            const char *error = PB_GET_ERROR(ostream);
            return false;
        }
        datapointptr = datapointptr->next;
    }

    return true;
}

// Protobuf encoding of a Metric definition
bool ScopeMetrics_encode_metric(pb_ostream_t *ostream, const pb_field_iter_t *field, void *const *arg)
{
    Metricptr metricptr = (Metricptr)(*arg);

    while (metricptr != NULL) {
        Metric metric = {};

        // Metric definition
        metric.name.arg = metricptr->name;
        metric.name.funcs.encode = encode_string;

        metric.description.arg = metricptr->description;
        metric.description.funcs.encode = encode_string;

        metric.unit.arg = metricptr->unit;
        metric.unit.funcs.encode = encode_string;

        /*
         * Metric type: counter (sum, monotonic)
         * A stateful counter would be cumulative, as it increments the value
         * Here we use a delta, to indicate to increment by 1 at each pulse
         */
        if (metricptr->datatype == METRIC_GAUGE) {
            metric.which_data = Metric_gauge_tag;
            metric.data.gauge.data_points.arg = metricptr->dpHead;
            metric.data.gauge.data_points.funcs.encode = Gauge_encode_data_points;
        } else if (metricptr->datatype == METRIC_SUM) {
            metric.which_data = Metric_sum_tag;
            metric.data.sum.is_monotonic = metricptr->monotonic;
            // Cumulative if we pass a persistent, incrementing value
            // metric.data.sum.aggregation_temporality = AggregationTemporality_AGGREGATION_TEMPORALITY_CUMULATIVE;
            // Delta if we pass the difference between now and before (e.g. value = 1)
            // metric.data.sum.aggregation_temporality = AggregationTemporality_AGGREGATION_TEMPORALITY_DELTA;
            metric.data.sum.aggregation_temporality = (AggregationTemporality)metricptr->aggregation;
            metric.data.gauge.data_points.arg = metricptr->dpHead;
            metric.data.gauge.data_points.funcs.encode = Sum_encode_data_points;
        }
        
        // Build the submessage tag
        if (!pb_encode_tag_for_field(ostream, field))
        {
            const char *error = PB_GET_ERROR(ostream);
            return false;
        }

        // Build the submessage payload
        if (!pb_encode_submessage(ostream, Metric_fields, &metric))
        {
            const char *error = PB_GET_ERROR(ostream);
            return false;
        }

        metricptr = metricptr->next;
    }

    return true;
}

// Protobuf encoding of a scope (passthrough - nothing much done here)
bool ResourceMetrics_encode_scope_metrics(pb_ostream_t *ostream, const pb_field_iter_t *field, void *const *arg)
{
    Metricptr metricptr = (Metricptr)(*arg);
    if (metricptr != NULL)
    {
        ScopeMetrics scope_metrics = {};

        scope_metrics.metrics.arg = metricptr;
        scope_metrics.metrics.funcs.encode = ScopeMetrics_encode_metric;

        // Build the submessage tag
        if (!pb_encode_tag_for_field(ostream, field))
        {
            const char *error = PB_GET_ERROR(ostream);
            return false;
        }

        // Build the submessage payload
        if (!pb_encode_submessage(ostream, ScopeMetrics_fields, &scope_metrics))
        {
            const char *error = PB_GET_ERROR(ostream);
            return false;
        }
    }

    return true;
}

// Protobuf encoding of the top level (resourcemetrics)
bool MetricsData_encode_resource_metrics(pb_ostream_t *ostream, const pb_field_iter_t *field, void *const *arg)
{
    Resourceptr resourceptr = (Resourceptr)(*arg);

    while (resourceptr != NULL)
    {
        ResourceMetrics resource_metrics = {};

        resource_metrics.has_resource = true;

        if (resourceptr->aHead != NULL)
        {
            resource_metrics.resource.attributes.arg = resourceptr->aHead;
            resource_metrics.resource.attributes.funcs.encode = KeyValue_encode_attributes;
        }

        if (resourceptr->mHead != NULL)
        {
            resource_metrics.scope_metrics.arg = resourceptr->mHead;
            resource_metrics.scope_metrics.funcs.encode = ResourceMetrics_encode_scope_metrics;
        }

        // Build the submessage tag
        if (!pb_encode_tag_for_field(ostream, field))
        {
            const char *error = PB_GET_ERROR(ostream);
        }

        // Build the submessage payload
        if (!pb_encode_submessage(ostream, ResourceMetrics_fields, &resource_metrics))
        {
            const char *error = PB_GET_ERROR(ostream);
            return false;
        }

        resourceptr = resourceptr->next;
    }
    return true;
}

/* The main() for protobuf:
 *   - data is assigned to the top-level encoding function
 *   - stream is created for the output, which points to a storage container
 *   - encoding is then performed (multiple passes, cascading thru encode methods)
 *   - data is written to output stream, along with size, status, etc
 *   - data is available in storage container
 */
size_t buildProtobuf(Resourceptr args, uint8_t *pbufPayload, size_t pbufMaxSize)
{
    MetricsData metricsdata = {};

    metricsdata.resource_metrics.arg = args;
    metricsdata.resource_metrics.funcs.encode = MetricsData_encode_resource_metrics;

    pb_ostream_t output = pb_ostream_from_buffer(pbufPayload, pbufMaxSize);
    int pbufStatus = pb_encode(&output, MetricsData_fields, &metricsdata);
    size_t pbufPayloadSize = output.bytes_written;

    // if there's a pbuf error
    if (!pbufStatus)
        return 0;

    return pbufPayloadSize;
}

// Get the system time (needs to be set via RTC/ NTP)
uint64_t getEpochNano(void)
{
    time_t epoch;
    time(&epoch);
    return (uint64_t) epoch * 1000000000;
}

void addResAttr(Resourceptr p, char *key, char *value) {
    // Safety check
    if (p != NULL) {
        Attrptr temp = (Attrptr)malloc(sizeof(Attrnode));
        temp->key = key;
        temp->value = value;
        temp->next = NULL;
        if (p->aTail == NULL) {
            // Set the new data set as the head & tail
            p->aTail = p->aHead = temp;
        } else {
            // Set the new data set as next on the last seen tail node
            p->aTail->next = temp;
            // Make the new data set the new tail node.
            p->aTail = temp;
        }
    }
}

void addDpAttr(Resourceptr p, char *key, char *value) {
    if (p != NULL) {
        Attrptr temp = (Attrptr)malloc(sizeof(Attrnode));
        temp->key = key;
        temp->value = value;
        temp->next = NULL;
        if (p->mTail->dpTail->aTail == NULL) {
            p->mTail->dpTail->aTail = p->mTail->dpTail->aHead = temp;
        } else {
            p->mTail->dpTail->aTail->next = temp;
            p->mTail->dpTail->aTail = temp;
        }
    }
}

void addDatapoint(Resourceptr p, int type, void *arg) {
    if (p != NULL) {
        Datapointptr temp = (Datapointptr)malloc(sizeof(Datapointnode));
        temp->aHead = NULL;
        temp->aTail = NULL;
        temp->time = getEpochNano();
        if (type == AS_DOUBLE) {
            temp->type = AS_DOUBLE;
            double* value = (double*)arg;
            temp->value.as_double = *value;
            free(value);
        } else {
            temp->type = AS_INT;
            int64_t* value = (int64_t*)arg;
            temp->value.as_int = *value;
            free(value);
        }
        temp->next = NULL;
        if (p->mTail->dpTail == NULL) {
            p->mTail->dpTail = p->mTail->dpHead = temp;
        } else {
            // Set the new dataset as next on the last seen tail node
            p->mTail->dpTail->next = temp;
            // Make the new dataset the new tail node.
            p->mTail->dpTail = temp;
        }
    }
}

// ptr, name, desc, unit, datatype, aggregation, is_monotonic
void addMetric(Resourceptr p, char *name, char *description, char *unit, int datatype, int aggregation, bool is_monotonic) {
    // Safety check
    if (p != NULL) {
        Metricptr temp = (Metricptr)malloc(sizeof(Metricnode));
        temp->name = name;
        temp->description = description;
        temp->unit = unit;
        temp->datatype = datatype;
        temp->monotonic = is_monotonic;
        temp->aggregation = aggregation;
        temp->dpHead = NULL;
        temp->dpTail = NULL;
        temp->next = NULL;
        if (p->mTail == NULL) {
            // Set the new data set as the head & tail
            p->mTail = p->mHead = temp;
        } else {
            // Set the new data set as next on the last seen tail node
            p->mTail->next = temp;
            // Make the new data set the new tail node.
            p->mTail = temp;
        }
    }
}

// Data structure functions - create (malloc)
Resourceptr addOteldata(void) {
    Resourceptr p = (Resourceptr)malloc(sizeof(Resourcenode));
    p->aHead = p->aTail = NULL;
    p->mHead = p->mTail = NULL;
    p->next = NULL;
    return p;
}

bool freeResAttr(Resourceptr p) {
    if (p != NULL) {
        Attrptr temp = NULL;
        while (p->aHead != NULL) {
            temp = p->aHead;
            p->aHead = p->aHead->next;
            free(temp);
        }
        p->aTail = NULL;
        return 0;
    } else {
        return 1;
    }
}


bool freeDpAttr(Datapointptr p) {
    if (p != NULL) {
        Attrptr temp = NULL;
        while (p->aHead != NULL) {
            temp = p->aHead;
            p->aHead = p->aHead->next;
            free(temp);
        }
        p->aTail = NULL;
        return 0;
    } else {
        return 1;
    }
}

bool freeDatapoint(Metricptr p) {
    if (p != NULL) {
        Datapointptr temp = NULL;
        while (p->dpHead != NULL) {
            temp = p->dpHead;
            p->dpHead = p->dpHead->next;
            freeDpAttr(temp);
            free(temp);
        }
        p->dpTail = NULL;
        return 0;
    } else {
        return 1;
    }
}

bool freeMetric(Resourceptr p) {
    if (p != NULL) {
        Metricptr temp = NULL;
        while (p->mHead != NULL) {
            temp = p->mHead;
            p->mHead = p->mHead->next;
            freeDatapoint(temp);
            free(temp);
        }
        p->mTail = NULL;
        return 0;
    } else {
        return 1;
    }
}

// Data structure functions - release (free memory)
void freeOteldata(Resourceptr ptr) {
    Resourceptr temp = NULL;
    while (ptr != NULL) {
        freeMetric(ptr);
        freeResAttr(ptr);
        temp = ptr;
        ptr = ptr->next;
        free(temp);
    }
}

void printOteldata(Resourceptr p) {
    Resourceptr resource = p;
    Metricptr metric = NULL;
    Datapointptr datapoint = NULL;
    Attrptr attr = NULL;

    while (resource != NULL) {
        Serial.println("-node-");
        attr = resource->aHead;
        while (attr != NULL) {
            Serial.print("\tResAttr: ");
            Serial.println(attr->key);
            attr = attr->next;
        }

        metric = resource->mHead;
        while (metric != NULL) {
            Serial.print("\tMetric: ");
            Serial.println(metric->name);
            datapoint = metric->dpHead;
            while (datapoint != NULL) {
                Serial.print("\t\tDatapoint: ");
                if (datapoint->type == AS_INT) {
                    Serial.println(datapoint->value.as_int);
                } else {
                    Serial.println(datapoint->value.as_double);
                }
                attr = datapoint->aHead;
                while (attr != NULL) {
                    Serial.print("\t\t\tAttr: ");
                    Serial.println(attr->value);
                    attr = attr->next;
                }
                datapoint = datapoint->next;
            }
            metric = metric->next;
        }
        resource = resource->next;
    }
}
