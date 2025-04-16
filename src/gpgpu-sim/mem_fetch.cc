// Copyright (c) 2009-2011, Tor M. Aamodt
// The University of British Columbia
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
// Redistributions in binary form must reproduce the above copyright notice, this
// list of conditions and the following disclaimer in the documentation and/or
// other materials provided with the distribution.
// Neither the name of The University of British Columbia nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "mem_fetch.h"
#include "mem_latency_stat.h"
#include "shader.h"
#include "visualizer.h"
#include "gpu-sim.h"

unsigned mem_fetch::sm_next_mf_request_uid=1;

mem_fetch::mem_fetch( const mem_access_t &access, 
                      const warp_inst_t *inst,
                      unsigned ctrl_size, 
                      unsigned wid,
                      unsigned sid, 
                      unsigned tpc, 
                      const class memory_config *config )
{
   m_request_uid = sm_next_mf_request_uid++;
   m_access = access;
   if( inst ) { 
       m_inst = *inst;
       assert( wid == m_inst.warp_id() );
   }
   m_data_size = access.get_size();
   m_ctrl_size = ctrl_size;
   m_sid = sid;
   m_tpc = tpc;
   m_wid = wid;
   config->m_address_mapping.addrdec_tlx(access.get_addr(),&m_raw_addr);
   m_partition_addr = config->m_address_mapping.partition_address(access.get_addr());
   m_type = m_access.is_write()?WRITE_REQUEST:READ_REQUEST;
   m_timestamp = gpu_sim_cycle + gpu_tot_sim_cycle;
   m_timestamp2 = 0;
   m_status = MEM_FETCH_INITIALIZED;
   m_status_change = gpu_sim_cycle + gpu_tot_sim_cycle;
   m_mem_config = config;
   icnt_flit_size = config->icnt_flit_size;

   m_dma = false;
   // yhyang {
   m_cxl_req_type = CXL_INVALID;
   m_cxl_ret_path = CXL_NONE;
   m_ndc_resp = NDC_INVALID;

   l2_done = false;
   dram_done = false;
   // } yhyang
}

//yhyang added for cloning
mem_fetch::mem_fetch(const mem_fetch &other)
{
    // m_request_uid = other.m_request_uid; // UID 복사
    m_request_uid = sm_next_mf_request_uid++;
    m_access = other.m_access;                 // 메모리 접근 정보 복사
    m_inst = other.m_inst;                     // 명령어 정보 복사
    m_data_size = other.m_data_size;           // 데이터 크기 복사
    m_ctrl_size = other.m_ctrl_size;           // 제어 크기 복사
    m_sid = other.m_sid;                       // 쉐이더 ID 복사
    m_tpc = other.m_tpc;                       // TPC ID 복사
    m_wid = other.m_wid;                       // 워프 ID 복사
    m_raw_addr = other.m_raw_addr;             // 주소 정보 복사
    m_partition_addr = other.m_partition_addr; // 파티션 주소 복사
    m_type = other.m_type;                     // 요청 타입 복사
    //m_timestamp = other.m_timestamp;           // 타임스탬프 복사
    m_timestamp = gpu_sim_cycle + gpu_tot_sim_cycle;
    m_timestamp2 = other.m_timestamp2;         // 타임스탬프 2 복사
    m_status = other.m_status;                 // 상태 복사
    m_status_change = other.m_status_change;   // 상태 변경 시간 복사
    m_mem_config = other.m_mem_config;         // 메모리 구성 복사
    icnt_flit_size = other.icnt_flit_size;     // 플릿 크기 복사
    m_dma = other.m_dma;                       // DMA 상태 복사

    m_cxl_req_type = other.m_cxl_req_type;
    m_cxl_ret_path = other.m_cxl_ret_path;
    m_ndc_resp = other.m_ndc_resp;

    l2_done = other.l2_done;
    dram_done = other.dram_done;
}


mem_fetch::~mem_fetch()
{
    m_status = MEM_FETCH_DELETED;
}

//move_to_memfetch.h:#define MF_TUP_BEGIN(X) static const char* Status_str[] = {
//move_to_memfetch.h:#define MF_TUP(X) #X
//move_to_memfetch.h:#define MF_TUP_END(X) };
//move_to_memfetch.h:#include "mem_fetch_status.tup"
//move_to_memfetch.h:#undef MF_TUP_BEGIN
//move_to_memfetch.h:#undef MF_TUP
//move_to_memfetch.h:#undef MF_TUP_END

void mem_fetch::print( FILE *fp, bool print_inst ) const
{
    if (this == NULL) {
        fprintf(fp, " <NULL mem_fetch pointer>\n");
        return;
    }
    fprintf(fp, "  mf: uid=%6u, sid%02u:w%02u, part=%u, ", m_request_uid, m_sid, m_wid, m_raw_addr.chip);
    m_access.print(fp);
    if ((unsigned)m_status < NUM_MEM_REQ_STAT)
        fprintf(fp, " status = %s (%llu), ", Status_str[m_status], m_status_change);
    else
        fprintf(fp, " status = %u??? (%llu), ", m_status, m_status_change);
    if (!m_mem_config->m_L3_NDC_config.disabled())
        fprintf(fp, " cxl_req_type = %s, cxl_return_path = %s, ndc_resp = %s, ", cxl_req_type_to_str(m_cxl_req_type), cxl_ret_path_to_str(m_cxl_ret_path), ndc_resp_type_to_str(m_ndc_resp));   //YH_DEBUG
        fprintf(fp, " l2_done = %d, dram_done = %d ", l2_done, dram_done);   //YH_DEBUG
    if (!m_inst.empty() && print_inst)
        m_inst.print(fp);
    else
        fprintf(fp, "\n");
}

void mem_fetch::set_status( enum mem_fetch_status status, unsigned long long cycle ) 
{
    if(status != m_status)
        if (get_request_uid() == TGT_UID) {
            printf("[YH_DEBUG][%d] Moved info. uid: %d, %s -> %s\n", cycle, get_request_uid(), Status_str[m_status], Status_str[status]);
            this->print(stdout);
        }
    m_status = status;
    m_status_change = cycle;
}

bool mem_fetch::isatomic() const
{
   if( m_inst.empty() ) return false;
   return m_inst.isatomic();
}

void mem_fetch::do_atomic()
{
    m_inst.do_atomic( m_access.get_warp_mask() );
}

bool mem_fetch::istexture() const
{
    if( m_inst.empty() ) return false;
    return m_inst.space.get_type() == tex_space;
}

bool mem_fetch::isconst() const
{ 
    if( m_inst.empty() ) return false;
    return (m_inst.space.get_type() == const_space) || (m_inst.space.get_type() == param_space_kernel);
}

/// Returns number of flits traversing interconnect. simt_to_mem specifies the direction
unsigned mem_fetch::get_num_flits(bool simt_to_mem){
	unsigned sz=0;
	// If atomic, write going to memory, or read coming back from memory, size = ctrl + data. Else, only ctrl
	if( isatomic() || (simt_to_mem && get_is_write()) || !(simt_to_mem || get_is_write()) )
		sz = size();
	else
		sz = get_ctrl_size();

	return (sz/icnt_flit_size) + ( (sz % icnt_flit_size)? 1:0);
}

// yhyang {
void mem_fetch::set_cxl_req_type(mf_cxl_req_type req_type) {
    m_cxl_req_type = req_type;
    return;
}
mf_cxl_req_type mem_fetch::get_cxl_req_type() const {
    return m_cxl_req_type;
}
void mem_fetch::set_cxl_ret_path(mf_cxl_ret_path ret_path) {
    m_cxl_ret_path = ret_path;
}
mf_cxl_ret_path mem_fetch::get_cxl_ret_path() const {
    return m_cxl_ret_path;
}
void mem_fetch::set_ndc_resp(mf_ndc_resp_type ndc_resp) {
    m_ndc_resp = ndc_resp;
    return;
}
mf_ndc_resp_type mem_fetch::get_ndc_resp() const {
    return m_ndc_resp;
}
const char *mem_fetch::cxl_req_type_to_str(mf_cxl_req_type type) const {
    if     (type == CXL_INVALID) return "CXL_INVALID";
    else if(type == CXL_RD_PREFETCH) return "CXL_RD_PREFETCH";
    else if(type == CXL_RD_PREDICT) return "CXL_RD_PREDICT";
    else if(type == CXL_RD_LINE_FILL) return "CXL_RD_LINE_FILL";
    else if(type == CXL_WR_LINE_FILL) return "CXL_WR_LINE_FILL";
    //yhyang:else if(type == CXL_WR_LINE_FILL_NEW) return "CXL_WR_LINE_FILL_NEW";
    else if(type == CXL_WB) return "CXL_WB";
    else return "CXL_REQ_TYPE_UNDEFINED";
}
const char *mem_fetch::cxl_ret_path_to_str(mf_cxl_ret_path type) const {
    if     (type == CXL_NONE) return "CXL_NONE";
    else if(type == CXL_L2) return "CXL_L2";
    else if(type == CXL_DRAM) return "CXL_DRAM";
    else if(type == CXL_L2_DRAM) return "CXL_L2_DRAM";
    else return "CXL_RET_PATH_UNDEFINED";
}
const char *mem_fetch::ndc_resp_type_to_str(mf_ndc_resp_type type) const {
    if     (type == NDC_INVALID) return "NDC_INVALID";
    else if(type == NDC_HIT) return "NDC_HIT";
    else if(type == NDC_MISS) return "NDC_MISS";
    //yhyang:else if(type == NDC_EVICT) return "NDC_EVICT";
    else if(type == NDC_FILL_DONE) return "NDC_FILL_DONE";
    else return "NDC_RESP_TYPE_UNDEFINED";
}
void mem_fetch::set_l2_done(bool tf) {
    l2_done = tf;
}
void mem_fetch::set_dram_done(bool tf) {
    dram_done = tf;
}
void mem_fetch::set_type_to_write() {
    m_type = WRITE_REQUEST;
    m_access.m_type = NDC_LINEFILL_W;  // TODO
    m_access.m_write = true;           // TODO
}
//} yhyang
