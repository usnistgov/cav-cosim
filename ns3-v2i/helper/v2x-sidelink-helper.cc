/*
 * NIST-developed software is provided by NIST as a public service. You may use,
 * copy, and distribute copies of the software in any medium, provided that you
 * keep intact this entire notice. You may improve, modify, and create
 * derivative works of the software or any portion of the software, and you may
 * copy and distribute such modifications or works. Modified works should carry
 * a notice stating that you changed the software and should note the date and
 * nature of any such change. Please explicitly acknowledge the National
 * Institute of Standards and Technology as the source of the software. 
 *
 * NIST-developed software is expressly provided "AS IS." NIST MAKES NO WARRANTY
 * OF ANY KIND, EXPRESS, IMPLIED, IN FACT, OR ARISING BY OPERATION OF LAW,
 * INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, NON-INFRINGEMENT, AND DATA ACCURACY. NIST
 * NEITHER REPRESENTS NOR WARRANTS THAT THE OPERATION OF THE SOFTWARE WILL BE
 * UNINTERRUPTED OR ERROR-FREE, OR THAT ANY DEFECTS WILL BE CORRECTED. NIST DOES
 * NOT WARRANT OR MAKE ANY REPRESENTATIONS REGARDING THE USE OF THE SOFTWARE OR
 * THE RESULTS THEREOF, INCLUDING BUT NOT LIMITED TO THE CORRECTNESS, ACCURACY,
 * RELIABILITY, OR USEFULNESS OF THE SOFTWARE.
 * 
 * You are solely responsible for determining the appropriateness of using and
 * distributing the software and you assume all risks associated with its use,
 * including but not limited to the risks and costs of program errors,
 * compliance with applicable laws, damage to or loss of data, programs or
 * equipment, and the unavailability or interruption of operation. This software 
 * is not intended to be used in any situation where a failure could cause risk
 * of injury or damage to property. The software developed by NIST employees is
 * not subject to copyright protection within the United States.
 *
 * Authors:
 *  Raphael Barbau
*/

#include "v2x-sidelink-helper.h"

NS_LOG_COMPONENT_DEFINE("V2XSidelinkHelper");

namespace ns3
{

V2XSidelinkHelper::V2XSidelinkHelper(Ipv4Address broadcastIpv4Address)
                              : broadcastIpv4Address(broadcastIpv4Address),
                              nrHelper(CreateObject<NrHelper>()),
                              nrSlHelper(CreateObject<NrSlHelper>()),
                              epcHelper(CreateObject<NrPointToPointEpcHelper>())
{
        NS_LOG_DEBUG("Creating V2X Sidelink helper");
    // BEG NEW SIDELINK CODE
    // Sidelink examples from https://gitlab.com/cttc-lena/nr/-/blob/nr-v2x-dev/examples/nr-v2x-examples/cttc-nr-v2x-demo-simple.cc?ref_type=heads
    
    // NETWORKING INTRO

    // 5G/New Radio network layers:
    // L0: PHY
    // L1: MAC: Media Access Control
    // L2: RLC: Radio Link Control
    // L3: PDCP: Packet Data Convergence Protocol
    // L4: RRC: Radio Resource Control
    // L5: NAS

    // Frequency subdivision: 
    // Frame: entire bandwidth
    // Resource block: 12 subcarriers
    // Subcarrier: length depends on numerology

    // Time subdivision:
    // Frame: 10ms
    // Subframe: 1ms
    // Slot: 6-7 OFDM symbol for LTE, 12-14 OFDM symbols for NR
    // OFDM symbol: length depends on numerology

    // Numerology: Subcarrier specing (mu): higher frequency -> higher number of slots
    // Resource Block: 1 slot in time and 12 subcarriers in frequency

    // Sidelink
    // Mode 1: inside eNodeB coverage area
    // Mode 2: outside eNodeB coverage area

    // Physical Sidelink Control Channel: for signalling traffic
    // Physical Sidelink Shared Channel: for data

    // 1. Configuration parameters
    
    // Radio Link Control, Unacklownedged Mode, maximum transmission buffer size
    // Service Data Units (SDUd) above get discarded
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // 2. Logging
    
    // In main code

    // 3. Helpers
    // Evolved Packet Core (EPC) / System Architecture Evolution (SAE) Core NETWORKS
    // Defines User Equipment, eNodeB/gNodeB, Mobility Management Entity (MME), etc.
    nrHelper->SetEpcHelper(epcHelper);


    // 4. Configuration of Operation band for sidelink
    // An Operation Band has 1+ Component carrier (CC), which has 1+ Bandwidth part
    CcBwpCreator ccBwpCreator;

    // Bandwidth part numerology mu 
    // Higher mu -> higher frequency -> lower duration
    // subcarrier spacing = 15*2^mu
    uint16_t numerologyBwpSl = 2;
    // Central frequency of bandwidth (DlEarfcn-UlEarfcn)
    // EARFCN: E-UTRA absolute radio-frequency channel number
    // E-UTRA: Evolved UMTS Terrestrial Radio Access
    // EARFCN (Uplink and Downlink) <-> Frequency + LTE band
    // see https://www.sqimway.com/lte_band.php
    // Using LTE band 47 (https://www.qualcomm.com/products/automotive/qualcomm-c-v2x-9150)
    // Uplink and Downlink have same frequency
    double centralFrequencyBandSl = 5.89e9;
    // Sidelink bandwidth, in multiple of 100 KHz: 400 = 40 MHz
    // n47: 10,20,30,40MHz?
    uint16_t bandwidthBandSl = 400;
    // Number of Component Carriers per band
    const uint8_t numCcPerBand = 1;

    // Scenario: do not seem to be used for anything!
    // RMa: LineOfSight/not
    // UMa: LineOfSight/not
    // UMi: LineOfSight/not
    // InH, OfficeOpen/Mixed, LineOfSight/not
    // V2V, Urban/Highway
    CcBwpCreator::SimpleOperationBandConf bandConfSl(centralFrequencyBandSl,
                                                     bandwidthBandSl,
                                                     numCcPerBand,
                                                     BandwidthPartInfo::V2V_Highway);

    bandSl = ccBwpCreator.CreateOperationBandContiguousCc(bandConfSl);
    allBwps = CcBwpCreator::GetAllBwps({bandSl});

    // Presumably ThreeGppPropagationLossModel
    // TODO: this might be where to add custom propagation losses
    // Channel coherence time?
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(100)));
    // Time between updated of channel condition (line of sight, indoor/outdoor, penetration loss)
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    // To disable shadow fading (?)
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    nrHelper->InitializeOperationBand(&bandSl);

    // 4. ??
    Packet::EnableChecking();
    Packet::EnablePrinting();

    // 5. Configuration for all nodes
    
    // Delay for S1-U (User plane) link, between eNodeB/gNodeB and Service Gateway (SGW)
    // Likely irrelevant for sidelink!
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    // User Equipment Antennas
    // Not sure if it matters
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    // Isotropic goes in all directions
    // TODO: could replace isotropic antenna with parabolic antenna (Beamwidth/Orientation)
    nrHelper->SetUeAntennaAttribute("AntennaElement", PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Tranmission power, in dBm
    // See https://www.qualcomm.com/products/automotive/qualcomm-c-v2x-9150
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(23));
    
    // Set User Equipment MAC type to Sidelink
    nrHelper->SetUeMacTypeId(NrSlUeMac::GetTypeId());
    // Random resource/slot selection, *not* based on sensing (PHY can sense sidelink resources)
    nrHelper->SetUeMacAttribute("EnableSensing", BooleanValue(false));
    // Resource/Slot selection window start
    nrHelper->SetUeMacAttribute("T1", UintegerValue(2));
    // Resource/Slot selection window end
    nrHelper->SetUeMacAttribute("T2", UintegerValue(33));
    // Active pool (??)
    nrHelper->SetUeMacAttribute("ActivePoolId", UintegerValue(0));

    // Set Bandwidth part manager type to Sidelink 
    nrHelper->SetBwpManagerTypeId(TypeId::LookupByName("ns3::NrSlBwpManagerUe"));

    // OLD code for MCPTT
    // uint8_t bwpIdForGbrMcptt = 0;
    // nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_MC_PUSH_TO_TALK", UintegerValue(bwpIdForGbrMcptt));
    // std::set<uint8_t> bwpIdContainer;
    // bwpIdContainer.insert(bwpIdForGbrMcptt);

    // NEW code for V2X
    // The bandwidth part manager algorithm returns an index for Quality of Service (QoS) Class Identifier (QCI)
    // Not sure is it should be GBR_V2X (guaranteed bit rate) or NGBR_V2X (non-guaranteed bit rate)
    uint8_t bwpIdForGbrV2x = 0;
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_V2X", UintegerValue(bwpIdForGbrV2x));
    bwpIdContainer.insert(bwpIdForGbrV2x);    


    // 6. Sidelink configuration

    nrSlHelper->SetEpcHelper(epcHelper);

    // Set error model (ns3::NrEesmCcT1, ns3::NrEesmCcT2, ns3::NrEesmIrT1, ns3::NrEesmIrT2, ns3::NrLteMiErrorModel)
    // Incremental Redundancy Hybrid ARQ (IR-HARQ) type 1
    std::string errorModel = "ns3::NrEesmIrT1";
    nrSlHelper->SetSlErrorModel(errorModel);
    // Adaptive Modulation and Coding (AMC): 
    // use the output of an error model to find the optimal Modulation and Coding Scheme (MCS)
    nrSlHelper->SetUeSlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));

    // Set scheduler (unclear if round robin, proportinal fair, or max carrier to interference)
    // Allocate the Physical resource blocks (PRBs) of a spectrum to UE in Transmission Time Intervals (TTI)
    // Communicate scheduling allocation (grant)
    nrSlHelper->SetNrSlSchedulerTypeId(NrSlUeMacSchedulerFixedMcs::GetTypeId());
    // modulation and coding scheme, see MCS index table for implications
    nrSlHelper->SetUeSlSchedulerAttribute("Mcs", UintegerValue(14));

    // 7. Service Access Point (SAP) definitions

    // Create a resource (block) pool factory
    Ptr<NrSlCommResourcePoolFactory> ptrFactory = Create<NrSlCommResourcePoolFactory>();

    // Resource pool factory parameters
    std::vector<std::bitset<1>> slBitmap = {1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1};
    // Sidelink time resources?
    ptrFactory->SetSlTimeResources(slBitmap);
    // Sensing window start time in ms
    ptrFactory->SetSlSensingWindow(100);
    // Number of slots in selection window
    ptrFactory->SetSlSelectionWindow(5);
    // Number of resource blocks allocated to physical sidelink control channel (PSCCH) transmission
    ptrFactory->SetSlFreqResourcePscch(10);
    // Number of resource blocks (RBs) per subchannel
    ptrFactory->SetSlSubchannelSize(50);
    // Maximum number of reserved PSCCH/PSSCH resources
    ptrFactory->SetSlMaxNumPerReserve(3);
    std::list<uint16_t> resourceReservePeriodList = {0, 100}; // in ms
    ptrFactory->SetSlResourceReservePeriodList(resourceReservePeriodList);
    // Once parameters are configured, create the resource pool definition
    LteRrcSap::SlResourcePoolNr slResourcePoolNr = ptrFactory->CreatePool();

    // The resource pool configuration definition refers to the previously created resource pool
    LteRrcSap::SlResourcePoolConfigNr slresoPoolConfigNr;
    slresoPoolConfigNr.haveSlResourcePoolConfigNr = true;
    uint16_t poolId = 0;
    LteRrcSap::SlResourcePoolIdNr slResourcePoolIdNr;
    slResourcePoolIdNr.id = poolId;
    slresoPoolConfigNr.slResourcePoolId = slResourcePoolIdNr;
    slresoPoolConfigNr.slResourcePool = slResourcePoolNr;

    // The Bandwidth part pool configuration definition refers to the resource pool configuration definition
    LteRrcSap::SlBwpPoolConfigCommonNr slBwpPoolConfigCommonNr;
    slBwpPoolConfigCommonNr.slTxPoolSelectedNormal[slResourcePoolIdNr.id] = slresoPoolConfigNr;

    // Bandwidth part definition 
    LteRrcSap::Bwp bwp;
    // Numerology used earlier
    bwp.numerology = numerologyBwpSl;
    // NR has 12-14 OFDM symbols
    bwp.symbolsPerSlots = 14;
    // resource block per resource block group
    bwp.rbPerRbg = 1;
    // Sidelink bandwidth used earlier
    bwp.bandwidth = bandwidthBandSl;

    // Sidelink bandwidth part definition
    LteRrcSap::SlBwpGeneric slBwpGeneric;
    slBwpGeneric.bwp = bwp;
    // add number of OFDM symbols
    slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum(14);
    // add start OFDM symbol
    slBwpGeneric.slStartSymbol = LteRrcSap::GetSlStartSymbolEnum(0);

    // Create Bandwidth configuration definition
    LteRrcSap::SlBwpConfigCommonNr slBwpConfigCommonNr;
    slBwpConfigCommonNr.haveSlBwpGeneric = true;
    slBwpConfigCommonNr.slBwpGeneric = slBwpGeneric;
    slBwpConfigCommonNr.haveSlBwpPoolConfigCommonNr = true;
    slBwpConfigCommonNr.slBwpPoolConfigCommonNr = slBwpPoolConfigCommonNr;

    // Create sidelink frequency configuration definitions
    LteRrcSap::SlFreqConfigCommonNr slFreConfigCommonNr;
    for (const auto& it : bwpIdContainer)
    {
        slFreConfigCommonNr.slBwpList[it] = slBwpConfigCommonNr;
    }

    // Create time division duplex configuration definition
    // Unclear what the F is for!!
    LteRrcSap::TddUlDlConfigCommon tddUlDlConfigCommon;
    tddUlDlConfigCommon.tddPattern = "DL|DL|DL|F|UL|UL|UL|UL|UL|UL|";

    // Create a general preconfiguration definition
    LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneralNr;
    slPreconfigGeneralNr.slTddConfig = tddUlDlConfigCommon;

    // Create UE preconfiguration definition
    LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
    // probability of keeping resource
    slUeSelectedPreConfig.slProbResourceKeep = 0;

    // Create Physical Sidelink Shared Channel (PSSCH) transmission parameter definitions
    LteRrcSap::SlPsschTxParameters psschParams;
    psschParams.slMaxTxTransNumPssch = 5;
    // Create Physical Sidelink Shared Channel (PSSCH) transmission configuration definitions
    LteRrcSap::SlPsschTxConfigList pscchTxConfigList;
    pscchTxConfigList.slPsschTxParameters[0] = psschParams;
    // Associate PSSCH configuration to UE preconfiguration
    slUeSelectedPreConfig.slPsschTxConfigList = pscchTxConfigList;

    // Create Sidelink preconfiguration
    slPreConfigNr.slPreconfigGeneral = slPreconfigGeneralNr;
    slPreConfigNr.slUeSelectedPreConfig = slUeSelectedPreConfig;
    slPreConfigNr.slPreconfigFreqInfoList[0] = slFreConfigCommonNr;

    // END NEW SIDELINK CODE

    // Set client/server addresses
    // MULTICAST: Ipv4Address groupAddress4("225.0.0.0");

    // Stores information about the sidelink channel
    // MULTICAST: slInfo.m_castType = SidelinkInfo::CastType::Groupcast;
    slInfo.m_castType = SidelinkInfo::CastType::Broadcast;
    // L2 ID?
    slInfo.m_dstL2Id = 255;
    // Resource Reservation Interval
    slInfo.m_rri = MilliSeconds(100);
    // Packet Delay Budget (PDB): maximum allowable delay between user equipment and Packet Data Network Gateway (P-GW)
    slInfo.m_pdb = Seconds(0);
    // Hybrid automatic repeat request (HARQ), for error control and correction: 
    slInfo.m_harqEnabled = true;


    // Traffic Flow Templates (TFT)
    tft = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL, broadcastIpv4Address, slInfo);
    

    // END NEW IP CONFIG


}

void V2XSidelinkHelper::Install(Ptr<Node> node) const
{
    NS_LOG_DEBUG("Installing V2X Sidelink");

    // Some functions require a NodeContainer...
    NodeContainer nodeContainer = NodeContainer(node);
    
    // Create net device container associated with all the bandwidth parts
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(nodeContainer, allBwps);

// Update configuration for each net device container
    for (auto it = ueNetDev.Begin(); it != ueNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    // Not sure what this does
    nrSlHelper->PrepareUeForSidelink(ueNetDev, bwpIdContainer);

    // Install Sidelink preconfiguration on all devices
    nrSlHelper->InstallNrSlPreConfiguration(ueNetDev, slPreConfigNr);


    // Install Internet stack (ARP, IPv4, ICMP, UDP, TCP, routing protocols) on all nodes
    stack.Install(node);

    
    // Assign IP addresses to all node network devices
    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(ueNetDev);

    // Set default gateway
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    // Set the default gateway for the UE
    Ptr<Ipv4StaticRouting> ueStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(node->GetObject<Ipv4>());
    // no backhaul / point to point / sidelink
    ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

    // Activation time for Sidelink bearers (???)
    nrSlHelper->ActivateNrSlBearer(Seconds(0.01), ueNetDev, tft);

}

} // namespace ns3