/*
Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the
disclaimer below) provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

    * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <aidl/android/hardware/gnss/IGnssAntennaInfoCallback.h>
#include "Gnss.h"
#include "GnssAntennaInfo.h"
#include <gps_extended_c.h>

namespace android {
namespace hardware {
namespace gnss {
namespace aidl {
namespace implementation {
struct AntennaCallbackState {
    std::mutex mutex;
    shared_ptr<IGnssAntennaInfoCallback> callback;
};
static std::weak_ptr<AntennaCallbackState> gAntennaInfo;
static std::mutex gAntennaInfoMutex;
using ::aidl::android::hardware::gnss::IGnss;

static void convertGnssAntennaInfo(std::vector<GnssAntennaInformation>& in,
        std::vector<IGnssAntennaInfoCallback::GnssAntennaInfo>& antennaInfos);

static void dispatchAntennaInfo(const shared_ptr<AntennaCallbackState>& state,
        std::vector<GnssAntennaInformation>& information) {
    shared_ptr<IGnssAntennaInfoCallback> callback;
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        callback = state->callback;
    }
    if (callback != nullptr) {
        std::vector<IGnssAntennaInfoCallback::GnssAntennaInfo> converted;
        convertGnssAntennaInfo(information, converted);
        auto result = callback->gnssAntennaInfoCb(converted);
        if (!result.isOk()) LOC_LOGw("Error antenna info cb");
    }
}

static void legacyAntennaInfoCb(std::vector<GnssAntennaInformation> information) {
    shared_ptr<AntennaCallbackState> state;
    {
        std::lock_guard<std::mutex> lock(gAntennaInfoMutex);
        state = gAntennaInfo.lock();
    }
    // No object pointer and no lock survives the outbound Binder call.
    if (state != nullptr) dispatchAntennaInfo(state, information);
}

static void convertGnssAntennaInfo(std::vector<GnssAntennaInformation>& in,
        std::vector<IGnssAntennaInfoCallback::GnssAntennaInfo>& out) {

    uint32_t vecSize, numberOfRows, numberOfColumns;
    vecSize = in.size();
    out.resize(vecSize);
    for (uint32_t i = 0; i < vecSize; i++) {
        out[i].carrierFrequencyHz = in[i].carrierFrequencyMHz * 1000000;
        out[i].phaseCenterOffsetCoordinateMillimeters.x =
                in[i].phaseCenterOffsetCoordinateMillimeters.x;
        out[i].phaseCenterOffsetCoordinateMillimeters.xUncertainty =
                in[i].phaseCenterOffsetCoordinateMillimeters.xUncertainty;
        out[i].phaseCenterOffsetCoordinateMillimeters.y =
                in[i].phaseCenterOffsetCoordinateMillimeters.y;
        out[i].phaseCenterOffsetCoordinateMillimeters.yUncertainty =
                in[i].phaseCenterOffsetCoordinateMillimeters.yUncertainty;
        out[i].phaseCenterOffsetCoordinateMillimeters.z =
                in[i].phaseCenterOffsetCoordinateMillimeters.z;
        out[i].phaseCenterOffsetCoordinateMillimeters.zUncertainty =
                in[i].phaseCenterOffsetCoordinateMillimeters.zUncertainty;

        numberOfRows = in[i].phaseCenterVariationCorrectionMillimeters.size();
        out[i].phaseCenterVariationCorrectionMillimeters.resize(numberOfRows);
        for (uint32_t j = 0; j < numberOfRows; j++) {
            numberOfColumns = in[i].phaseCenterVariationCorrectionMillimeters[j].size();
            out[i].phaseCenterVariationCorrectionMillimeters[j].row.resize(numberOfColumns);
            for (uint32_t k = 0; k < numberOfColumns; k++) {
                out[i].phaseCenterVariationCorrectionMillimeters[j].row[k] =
                        in[i].phaseCenterVariationCorrectionMillimeters[j][k];
            }
        }

        numberOfRows = in[i].phaseCenterVariationCorrectionUncertaintyMillimeters.size();
        out[i].phaseCenterVariationCorrectionUncertaintyMillimeters.resize(numberOfRows);
        for (uint32_t j = 0; j < numberOfRows; j++) {
            numberOfColumns = in[i].phaseCenterVariationCorrectionUncertaintyMillimeters[j].size();
            out[i].phaseCenterVariationCorrectionUncertaintyMillimeters[j].
                    row.resize(numberOfColumns);
            for (uint32_t k = 0; k < numberOfColumns; k++) {
                out[i].phaseCenterVariationCorrectionUncertaintyMillimeters[j].row[k] =
                        in[i].phaseCenterVariationCorrectionUncertaintyMillimeters[j][k];
            }
        }

        numberOfRows = in[i].signalGainCorrectionDbi.size();
        out[i].signalGainCorrectionDbi.resize(numberOfRows);
        for (uint32_t j = 0; j < numberOfRows; j++) {
            numberOfColumns = in[i].signalGainCorrectionDbi[j].size();
            out[i].signalGainCorrectionDbi[j].row.resize(numberOfColumns);
            for (uint32_t k = 0; k < numberOfColumns; k++) {
                out[i].signalGainCorrectionDbi[j].row[k] = in[i].signalGainCorrectionDbi[j][k];
            }
        }

        numberOfRows = in[i].signalGainCorrectionUncertaintyDbi.size();
        out[i].signalGainCorrectionUncertaintyDbi.resize(numberOfRows);
        for (uint32_t j = 0; j < numberOfRows; j++) {
            numberOfColumns = in[i].signalGainCorrectionUncertaintyDbi[j].size();
            out[i].signalGainCorrectionUncertaintyDbi[j].row.resize(numberOfColumns);
            for (uint32_t k = 0; k < numberOfColumns; k++) {
                out[i].signalGainCorrectionUncertaintyDbi[j].row[k] =
                        in[i].signalGainCorrectionUncertaintyDbi[j][k];
            }
        }
    }
}

void gnssAntennaInfoServiceDied(void* cookie) {
    LOC_LOGe("IGnssAntennaInfo AIDL service died");
    // The death cookie is intentionally null.  Binder does not promise that
    // unlink synchronizes a raced death callback, so retaining a raw State*
    // here would permit a use-after-free during extension teardown.  The
    // normal close/destructor path clears the shared callback state safely.
    (void)cookie;
}
GnssAntennaInfo::GnssAntennaInfo(Gnss* gnss) : mGnss(gnss),
    mDeathRecipient(AIBinder_DeathRecipient_new(&gnssAntennaInfoServiceDied)) {
    mCallbackState = std::make_shared<AntennaCallbackState>();
}

GnssAntennaInfo::~GnssAntennaInfo() {
    {
        std::lock_guard<std::mutex> lock(gAntennaInfoMutex);
        if (gAntennaInfo.lock() == mCallbackState) {
            gAntennaInfo.reset();
        }
    }
    close();
    if (mDeathRecipient != nullptr) {
        AIBinder_DeathRecipient_delete(mDeathRecipient);
        mDeathRecipient = nullptr;
    }
}

ScopedAStatus GnssAntennaInfo::setCallback(
        const shared_ptr<IGnssAntennaInfoCallback>& callback) {
    if (mGnss == nullptr) {
        LOC_LOGe("]: mGnss is nullptr");
        return ScopedAStatus::fromExceptionCode(IGnss::ERROR_GENERIC);
    }

    if (callback == nullptr) {
        return close();
    }

    std::unique_lock<std::mutex> lock(mCallbackState->mutex);
    if (mCallbackState->callback != nullptr) {
        AIBinder_unlinkToDeath(mCallbackState->callback->asBinder().get(), mDeathRecipient,
                nullptr);
    }

    mCallbackState->callback = callback;
    if (mCallbackState->callback != nullptr) {
        AIBinder_linkToDeath(mCallbackState->callback->asBinder().get(), mDeathRecipient,
                nullptr);
    }
    lock.unlock();

    if (mGnss->getGnssInterface() == nullptr) {
        close();
        return ScopedAStatus::fromExceptionCode(IGnss::ERROR_GENERIC);
    }
    {
        std::lock_guard<std::mutex> registryLock(gAntennaInfoMutex);
        gAntennaInfo = mCallbackState;
    }
    uint32_t result = mGnss->getGnssInterface()->antennaInfoInit(legacyAntennaInfoCb);
    if (result != ANTENNA_INFO_SUCCESS) {
        close();
        return ScopedAStatus::fromExceptionCode(IGnss::ERROR_GENERIC);
    }
    return ScopedAStatus::ok();
}
ScopedAStatus GnssAntennaInfo::close() {
    if (mGnss == nullptr) {
        LOC_LOGe("]: mGnss is nullptr");
        return ScopedAStatus::fromExceptionCode(STATUS_INVALID_OPERATION);
    }

    {
        std::lock_guard<std::mutex> registryLock(gAntennaInfoMutex);
        if (gAntennaInfo.lock() == mCallbackState) gAntennaInfo.reset();
    }
    std::unique_lock<std::mutex> lock(mCallbackState->mutex);
    if (mCallbackState->callback != nullptr) {
        AIBinder_unlinkToDeath(mCallbackState->callback->asBinder().get(), mDeathRecipient,
                nullptr);
        mCallbackState->callback = nullptr;
    }
    lock.unlock();
    if (mGnss->getGnssInterface() != nullptr) {
        mGnss->getGnssInterface()->antennaInfoClose();
    }
    return ScopedAStatus::ok();

}

void GnssAntennaInfo::gnssAntennaInfoCb
        (std::vector<GnssAntennaInformation>& gnssAntennaInformations) {

    dispatchAntennaInfo(mCallbackState, gnssAntennaInformations);
}

}
}  // namespace aidl
}  // namespace gnss
}  // namespace hardware
}  // namespace android
