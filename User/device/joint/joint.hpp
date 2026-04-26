#pragma once

#include <stdint.h>
#include <cmath>
#include <cstring>

#include "component/arm_lib/model/joint_mapping.h"
#include "component/user_math.h"
#include "device/motor/motor.hpp"

namespace mrobot {

// Reuse the arm_lib chain-model joint descriptors so the runtime layer and
// kinematic model share the same transmission and coupling semantics.
using JointType = arm_lib::ChainJointType;
using JointCouplingTerm = arm_lib::ChainJointCouplingTerm;
using JointTransmission = arm_lib::ChainTransmission;

struct ActuatorState {
    float position;
    float velocity;
    float torque;
    bool online;

    ActuatorState() : position(0.0f), velocity(0.0f), torque(0.0f), online(false) {}
};

class IActuator {
public:
    virtual ~IActuator() = default;

    virtual int8_t Register() = 0;
    virtual int8_t Enable() = 0;
    virtual int8_t Update() = 0;
    virtual int8_t Relax() = 0;
    virtual int8_t CommitControl() = 0;
    virtual bool HasPendingControl() const = 0;
    virtual void ClearPendingControl() = 0;
    virtual int8_t SetMIT(float position, float velocity, float kp, float kd, float torque_ff) = 0;
    virtual int8_t SetTorque(float torque) = 0;
    virtual ActuatorState GetState() const = 0;
};

template <typename MotorType>
class MotorActuatorAdapter final : public IActuator {
public:
    MotorActuatorAdapter() : motor_(nullptr) {}
    explicit MotorActuatorAdapter(MotorType* motor) : motor_(motor) {}

    void SetMotor(MotorType* motor) { motor_ = motor; }
    MotorType* GetMotor() const { return motor_; }

    int8_t Register() override { return motor_ ? motor_->Register() : -1; }
    int8_t Enable() override { return motor_ ? motor_->Enable() : -1; }
    int8_t Update() override { return motor_ ? motor_->Update() : -1; }
    int8_t Relax() override { return motor_ ? motor_->Disable() : -1; }
    int8_t CommitControl() override { return motor_ ? motor_->CommitCommand() : -1; }
    bool HasPendingControl() const override { return motor_ ? motor_->HasPendingCommand() : false; }
    void ClearPendingControl() override {
        if (motor_) {
            motor_->ClearPendingCommand();
        }
    }
    int8_t SetMIT(float position, float velocity, float kp, float kd, float torque_ff) override {
        return motor_ ? motor_->SetMIT(position, velocity, kp, kd, torque_ff) : -1;
    }
    int8_t SetTorque(float torque) override { return motor_ ? motor_->SetTorque(torque) : -1; }
    ActuatorState GetState() const override {
        ActuatorState state;
        if (!motor_) {
            return state;
        }
        const mrobot::motor::MotorState motor_state = motor_->GetState();
        state.position = motor_state.position_rad;
        state.velocity = motor_state.velocity_rad_s;
        state.torque = motor_state.torque_nm;
        state.online = motor_state.online;
        return state;
    }

private:
    MotorType* motor_;
};

struct JointControlParams {
    float qmin;
    float qmax;
    float kp;
    float kd;
};

class IJoint {
public:
    virtual ~IJoint() = default;

    virtual uint8_t GetId() const = 0;
    virtual const char* GetName() const = 0;
    virtual JointType GetType() const = 0;
    virtual float GetCurrentAngle() const = 0;
    virtual float GetCurrentVelocity() const = 0;
    virtual float GetTargetAngle() const = 0;
    virtual bool IsOnline() const = 0;
    virtual const JointControlParams& GetParams() const = 0;
    virtual float GetOffset() const = 0;
    virtual float GetFeedforwardTorque() const = 0;

    virtual void SetOffset(float offset) = 0;
    virtual void SetTargetAngle(float target) = 0;
    virtual void SetFeedforwardTorque(float torque) = 0;

    virtual int8_t Register() = 0;
    virtual int8_t Enable() = 0;
    virtual int8_t Update() = 0;
    virtual int8_t Relax() = 0;
    virtual int8_t CommitControl() = 0;
    virtual bool HasPendingControl() const = 0;
    virtual void ClearPendingControl() = 0;
    virtual int8_t PositionControl(float target_angle, float dt) = 0;
    virtual int8_t TorqueControl(float torque) = 0;
    virtual bool IsReached(float tolerance) const = 0;
};

class JointT final : public IJoint {
public:
        static constexpr uint8_t kJointNameMaxLen = 24U;

    JointT(uint8_t id, IActuator* actuator, const JointControlParams& params,
                     float q_offset, float freq,
                     const char* name = nullptr,
             JointType type = JointType::kRevolute)
        : id_(id),
      actuator_(actuator),
          params_(params),
          q_offset_(q_offset),
                    type_(type),
          transmission_() {
        (void)freq;
                SetName(name);
        for (uint8_t i = 0; i < ARM_LIB_MAX_JOINT_COUPLINGS; ++i) {
            coupling_sources_[i] = nullptr;
        }
        state_.current_angle = 0.0f;
        state_.current_velocity = 0.0f;
        state_.current_torque = 0.0f;
        state_.target_angle = 0.0f;
        state_.feedforward_torque = 0.0f;
        state_.online = false;
    }

    JointT(const JointT&) = delete;
    JointT& operator=(const JointT&) = delete;

    uint8_t GetId() const override { return id_; }
    const char* GetName() const override { return name_; }
    JointType GetType() const override { return type_; }
    float GetCurrentAngle() const override { return state_.current_angle; }
    float GetCurrentVelocity() const override { return state_.current_velocity; }
    float GetTargetAngle() const override { return state_.target_angle; }
    bool IsOnline() const override { return state_.online; }
    const JointControlParams& GetParams() const override { return params_; }
    float GetOffset() const override { return q_offset_; }
    float GetFeedforwardTorque() const override { return state_.feedforward_torque; }

    void SetName(const char* name) {
        if (name == nullptr) {
            name_[0] = '\0';
            return;
        }

        std::strncpy(name_, name, kJointNameMaxLen - 1U);
        name_[kJointNameMaxLen - 1U] = '\0';
    }

    void SetOffset(float offset) override { q_offset_ = offset; }

    void SetTargetAngle(float target) override {
        state_.target_angle = target;
    }

    void SetFeedforwardTorque(float torque) override {
        state_.feedforward_torque = torque;
    }

    void SetTransmission(const JointTransmission& transmission) {
        transmission_ = transmission;
    }

    const JointTransmission& GetTransmission() const {
        return transmission_;
    }

    bool SetCouplingSource(uint8_t term_index, const IJoint* source_joint) {
        if (term_index >= ARM_LIB_MAX_JOINT_COUPLINGS) {
            return false;
        }
        coupling_sources_[term_index] = source_joint;
        if (term_index >= transmission_.coupling_count) {
            transmission_.coupling_count = static_cast<uint8_t>(term_index + 1U);
        }
        transmission_.couplings[term_index].source_joint =
            (source_joint != nullptr) ? source_joint->GetId() : 0U;
        transmission_.couplings[term_index].enabled = (source_joint != nullptr);
        return true;
    }

    bool AddCoupling(const IJoint* source_joint, float coefficient) {
        const uint8_t source_joint_id =
            (source_joint != nullptr) ? source_joint->GetId() : 0U;
        if (!transmission_.add_coupling(source_joint_id, coefficient)) {
            return false;
        }
        const uint8_t index = static_cast<uint8_t>(transmission_.coupling_count - 1U);
        coupling_sources_[index] = source_joint;
        transmission_.couplings[index].enabled = (source_joint != nullptr);
        return true;
    }

    void SetCoupledJoint(const IJoint* joint) {
        transmission_.clear_couplings();
        for (uint8_t i = 0; i < ARM_LIB_MAX_JOINT_COUPLINGS; ++i) {
            coupling_sources_[i] = nullptr;
        }
        if (joint != nullptr) {
            AddCoupling(joint, 1.0f);
        }
    }

    int8_t Register() override {
        return actuator_ ? actuator_->Register() : -1;
    }

    int8_t Enable() override {
        return actuator_ ? actuator_->Enable() : -1;
    }

    int8_t Update() override {
        const int8_t ret = actuator_ ? actuator_->Update() : -1;
        const ActuatorState actuator_state = actuator_ ? actuator_->GetState() : ActuatorState();

        float joint_angle = DecodeJointPosition(actuator_state.position);
        joint_angle = NormalizeJointAngle(joint_angle);

        state_.current_angle = joint_angle;
        state_.current_velocity = DecodeJointVelocity(actuator_state.velocity);
        state_.current_torque = actuator_state.torque;
        state_.online = actuator_state.online;
        return ret;
    }

    int8_t Relax() override {
        return actuator_ ? actuator_->Relax() : -1;
    }

    int8_t CommitControl() override {
        return actuator_ ? actuator_->CommitControl() : -1;
    }

    bool HasPendingControl() const override {
        return actuator_ ? actuator_->HasPendingControl() : false;
    }

    void ClearPendingControl() override {
        if (actuator_) {
            actuator_->ClearPendingControl();
        }
    }

    int8_t PositionControl(float target_angle, float dt) override {
        (void)dt;
        if (target_angle < params_.qmin || target_angle > params_.qmax) {
            return -1;
        }

        state_.target_angle = target_angle;

        float kp = params_.kp;
        float kd = params_.kd;
        if (kp <= 0.0f) {
            kp = (id_ < 3U) ? 10.0f : 50.0f;
        }
        if (kd <= 0.0f) {
            kd = (id_ < 3U) ? 0.5f : 3.0f;
        }

        return actuator_ ? actuator_->SetMIT(
            EncodeActuatorPosition(target_angle),
            0.0f,
            kp,
            kd,
            state_.feedforward_torque) : -1;
    }

    int8_t TorqueControl(float torque) override {
        state_.feedforward_torque = torque;
        return actuator_ ? actuator_->SetTorque(torque) : -1;
    }

    bool IsReached(float tolerance) const override {
        return std::fabs(state_.target_angle - state_.current_angle) < tolerance;
    }

private:
    struct JointStateCache {
        float current_angle;
        float current_velocity;
        float current_torque;
        float target_angle;
        float feedforward_torque;
        bool online;
    };

    float SumCoupledJointAngles() const {
        float sum = 0.0f;
        for (uint8_t i = 0; i < transmission_.coupling_count; ++i) {
            if (!transmission_.couplings[i].enabled || coupling_sources_[i] == nullptr) {
                continue;
            }
            sum += transmission_.couplings[i].coefficient *
                   coupling_sources_[i]->GetCurrentAngle();
        }
        return sum;
    }

    float SumCoupledJointVelocities() const {
        float sum = 0.0f;
        for (uint8_t i = 0; i < transmission_.coupling_count; ++i) {
            if (!transmission_.couplings[i].enabled || coupling_sources_[i] == nullptr) {
                continue;
            }
            sum += transmission_.couplings[i].coefficient *
                   coupling_sources_[i]->GetCurrentVelocity();
        }
        return sum;
    }

    float DecodeJointPosition(float actuator_position) const {
        return arm_lib::chain_transmission_actuator_to_joint_position(
            transmission_, actuator_position, SumCoupledJointAngles(), q_offset_);
    }

    float DecodeJointVelocity(float actuator_velocity) const {
        return arm_lib::chain_transmission_actuator_to_joint_velocity(
            transmission_, actuator_velocity, SumCoupledJointVelocities());
    }

    float EncodeActuatorPosition(float joint_position) const {
        return arm_lib::chain_transmission_joint_to_actuator_position(
            transmission_, joint_position, SumCoupledJointAngles(), q_offset_);
    }

    float NormalizeJointAngle(float angle) const {
        const bool multi_turn = (params_.qmax - params_.qmin) > 2.0f * M_PI;
        if (multi_turn) {
            return angle;
        }
        if (angle > M_PI) {
            angle -= 2.0f * M_PI;
        }
        if (angle < -M_PI) {
            angle += 2.0f * M_PI;
        }
        return angle;
    }

    uint8_t id_;
    IActuator* actuator_;
    JointControlParams params_;
    float q_offset_;
    JointType type_;
    char name_[kJointNameMaxLen];
    JointTransmission transmission_;
    const IJoint* coupling_sources_[ARM_LIB_MAX_JOINT_COUPLINGS];
    JointStateCache state_;
};

using DmActuator = MotorActuatorAdapter<mrobot::motor::DmJ4310Motor>;
using LzActuator = MotorActuatorAdapter<mrobot::motor::LzRso3Motor>;
using DmJoint = JointT;
using LzJoint = JointT;

} // namespace mrobot
