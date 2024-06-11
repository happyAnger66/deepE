#include <glog/logging.h>
#include <ros/console.h>
#include <stdlib.h>
#include <utils/libsensors_chip.h>

#include <boost/foreach.hpp>

#define NAME_BUFFER_SIZE 250

SensorChip::SensorChip(sensors_chip_name const *chip_name)
    : internal_name_(chip_name) {
  char name_buffer[NAME_BUFFER_SIZE];
  sensors_snprintf_chip_name(name_buffer, NAME_BUFFER_SIZE, internal_name_);
  name_ = name_buffer;

  ROS_DEBUG("Found Sensor: %s", getName().c_str());

  // enumerate the features of this sensor
  sensors_feature const *feature;
  int number = 0;

  while ((feature = sensors_get_features(internal_name_, &number)) != NULL) {
    sensors_feature_type type = feature->type;
    SensorChipFeaturePtr feature_ptr;
    if (type == SENSORS_FEATURE_TEMP) {
      feature_ptr.reset(new TempSensor(*this, feature));
    } else {
      continue;
    }
    features_.push_back(feature_ptr);
  }
}

SensorChipFeature::SensorChipFeature(const SensorChip &chip,
                                     sensors_feature const *feature)
    : chip_(chip), feature_(feature) {
  name_ = feature_->name;
  char *label_c_str = sensors_get_label(chip_.internal_name_, feature_);
  if (label_c_str == NULL) {
    ROS_WARN("Could not get label for %s", name_.c_str());
    label_ = "(None)";
  } else {
    label_ = label_c_str;
    free(label_c_str);
  }
  full_name_ = getChipName() + "/" + getFeatureName();
  full_label_ = getChipName() + "/" + getFeatureLabel();

  ROS_DEBUG("\tFound Feature: %s(%s)[%d]", getFullLabel().c_str(),
            getFullName().c_str(), feature_->type);

  enumerate_subfeatures();
}

void SensorChipFeature::enumerate_subfeatures() {
  sensors_subfeature const *subfeature;
  int number = 0;

  while ((subfeature = sensors_get_all_subfeatures(
              chip_.internal_name_, feature_, &number)) != NULL) {
    SensorChipSubFeaturePtr subfeature_ptr(
        new SensorChipSubFeature(*this, subfeature));
    sub_features_.push_back(subfeature_ptr);
  }
}

SensorChipSubFeaturePtr SensorChipFeature::getSubFeatureByType(
    sensors_subfeature_type type) {
  BOOST_FOREACH (SensorChipSubFeaturePtr subfeature, sub_features_) {
    if (subfeature->getType() == type) return subfeature;
  }
  return SensorChipSubFeaturePtr();
}

SensorChipSubFeature::SensorChipSubFeature(const SensorChipFeature &feature,
                                           sensors_subfeature const *subfeature)
    : feature_(feature), subfeature_(subfeature), name_(subfeature->name) {
  ROS_DEBUG("\t\tFound Sub-Feature: %s[%d] = %f", getName().c_str(),
            subfeature_->type, getValue());
}

double SensorChipSubFeature::getValue() {
  double value;
  if (sensors_get_value(feature_.chip_.internal_name_, subfeature_->number,
                        &value) != 0) {
    ROS_WARN_STREAM("Failed to get value for " << feature_.getFullName() << " "
                                               << getName());
  }
  return value;
}

void TempSensor::buildStatus(cargo::proto::SystemInfo &msg) {
  SensorChipSubFeaturePtr temp =
      getSubFeatureByType(SENSORS_SUBFEATURE_TEMP_INPUT);
  SensorChipSubFeaturePtr max_temp =
      getSubFeatureByType(SENSORS_SUBFEATURE_TEMP_MAX);
  SensorChipSubFeaturePtr temp_crit =
      getSubFeatureByType(SENSORS_SUBFEATURE_TEMP_CRIT);
  SensorChipSubFeaturePtr temp_crit_alarm =
      getSubFeatureByType(SENSORS_SUBFEATURE_TEMP_CRIT_ALARM);
  if (temp) {
    auto *cpu_temperature = msg.add_cpu_temp();
    cpu_temperature->set_description(temp->getFeature().getFeatureLabel());
    cpu_temperature->set_temperature(temp->getValue());
    if (max_temp && max_temp->getValue() != 0)
      cpu_temperature->set_high(max_temp->getValue());
    if (temp_crit && temp_crit->getValue() != 0)
      cpu_temperature->set_crit(temp_crit->getValue());
  } else {
    LOG(WARNING) << temp->getName() << " get temperature failed!!";
  }
}
