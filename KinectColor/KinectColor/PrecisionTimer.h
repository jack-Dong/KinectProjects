// Author  : DustPG
// License : MIT: see more in "License.txt"

// ��;: 

// PrecisionTimer�� �߾��ȼ�ʱ�� �������ڶ�ʱ���ʱ ����:��s����
//		����֡��ʱ�䡢ĳ������ʱ�����ڳ�ʱ���ʱ����Ϊ����ԭ��ɥʧ����
//		�ϻ�����֧�� �»�����ˡ���Ƶ  ��֮ ����

#pragma once //��֤ͷ�ļ�ֻ������һ��

// �߾��ȼ�ʱ��
class PrecisionTimer
{
public:
    // ����Ƶ�� CPU��Ƶ�Ļ�Ӱ���Ĵ� �����ֶ�ˢ��һ�Σ�����ÿ2�����һ��
    void					RefreshFrequency(){ QueryPerformanceFrequency(&m_cpuFrequency); }
    // ��ʼ��ʱ
    void					Start(){ get_performance_counter(&m_cpuCounterStart); }
    // ��ʼʱ�丳ֵΪ����ʱ�� ����ʹ��ʱ��������
    void					MovStartEnd(){ m_cpuCounterStart = m_cpuCounterEnd; }
    // ����ʱ��(��) ���ص�����
    float					DeltaF_s(){
        get_performance_counter(&m_cpuCounterEnd);
        return (float)(m_cpuCounterEnd.QuadPart - m_cpuCounterStart.QuadPart)*1e-3F / (float)m_cpuFrequency.QuadPart;
    }
    // ����ʱ��(��) ����˫����
    double					DeltaD_s(){
        get_performance_counter(&m_cpuCounterEnd);
        return (double)(m_cpuCounterEnd.QuadPart - m_cpuCounterStart.QuadPart)*1e-3 / (double)m_cpuFrequency.QuadPart;
    }
    // ����ʱ��(����) ���ص�����
    float					DeltaF_ms(){
        get_performance_counter(&m_cpuCounterEnd);
        return (float)(m_cpuCounterEnd.QuadPart - m_cpuCounterStart.QuadPart) / (float)m_cpuFrequency.QuadPart;
    }
    // ����ʱ��(����) ����˫����
    double					DeltaD_ms(){
        get_performance_counter(&m_cpuCounterEnd);
        return (double)(m_cpuCounterEnd.QuadPart - m_cpuCounterStart.QuadPart) / (double)m_cpuFrequency.QuadPart;
    }
    // ����ʱ��(΢��) ���ص�����
    float					DeltaF_mcs(){
        get_performance_counter(&m_cpuCounterEnd);
        return (float)(m_cpuCounterEnd.QuadPart - m_cpuCounterStart.QuadPart)*1e3F / (float)m_cpuFrequency.QuadPart;
    }
    // ����ʱ��(΢��) ����˫����
    double					DeltaD_mcs(){
        get_performance_counter(&m_cpuCounterEnd);
        return (double)(m_cpuCounterEnd.QuadPart - m_cpuCounterStart.QuadPart)*1e3 / (double)m_cpuFrequency.QuadPart;
    }
private:
    // CPU ��ǰƵ��
    LARGE_INTEGER			m_cpuFrequency;
    // CPU ��ʼ��ʱʱ��
    LARGE_INTEGER			m_cpuCounterStart;
    // CPU ������ʱʱ��
    LARGE_INTEGER			m_cpuCounterEnd;
private:
    // ��ȡ CPU ִ�м��� QueryPerformanceCounter�ĸ����ܰ�
    void get_performance_counter(LARGE_INTEGER* counter){
        DWORD dwLow, dwHigh;
        __asm
        {
            rdtsc;
            mov dwLow, eax;
            mov dwHigh, edx;
        }
        counter->HighPart = dwHigh;
        counter->LowPart = dwLow;
    }
public:
    // ǿ�Ƶ�ʵ��
    PrecisionTimer(){ RefreshFrequency(); }
    // Ψһʵ��
    //static	PrecisionTimer	s_instance;
};