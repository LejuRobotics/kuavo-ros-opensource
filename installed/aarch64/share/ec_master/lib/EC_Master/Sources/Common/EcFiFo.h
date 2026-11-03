/** ---------------------------------------------------------------------------
 * \file
 * \author      Stefan Zintgraf
 * \copyright   acontis technologies GmbH, Weingarten, Germany
 *---------------------------------------------------------------------------*/

#ifndef INC_ECFIFO
#define INC_ECFIFO

/*-CFiFoList-----------------------------------------------------------------*/

typedef struct _EC_T_FIFO_DESC
{
    EC_T_DWORD dwFirst;
    EC_T_DWORD dwLast;
    EC_T_DWORD dwBufSize;
} EC_T_FIFO_DESC, *EC_PT_FIFO_DESC;

template <class VALUE>
class CFiFoList
{
protected:
    VALUE*                   m_pFiFoEntrys;
    volatile EC_PT_FIFO_DESC m_pDesc;
    EC_T_VOID*               m_poLock;

public:
    CFiFoList(EC_PT_FIFO_DESC pDesc, VALUE* pFiFoEntrys, EC_T_VOID* poLock, const EC_T_CHAR* szName)
    {
        EC_UNREFPARM(szName);
        m_pDesc         = pDesc;
        m_pFiFoEntrys   = pFiFoEntrys;
        m_poLock        = poLock;
    }
    CFiFoList(const CFiFoList&) {m_pFiFoEntrys = m_poLock = EC_NULL; OsDbgAssert(EC_FALSE);} /* objects are not copied */
    virtual ~CFiFoList() {}

    EC_T_VOID Lock(EC_T_VOID)
    {
        OsLock(m_poLock);
    }
    EC_T_VOID Unlock(EC_T_VOID)
    {
        OsUnlock(m_poLock);
    }

    /********************************************************************************/
    /**
     * \brief Get amount of added entries.
     *
     * \return amount of added entries
     */
    EC_T_DWORD GetCount(EC_T_VOID) const
    {
        EC_PT_FIFO_DESC pDesc = m_pDesc;

        if ((EC_NULL == pDesc) || (0 == pDesc->dwBufSize))
        {
            return 0;
        }
        return (EC_T_DWORD)(pDesc->dwLast + pDesc->dwBufSize - pDesc->dwFirst) % m_pDesc->dwBufSize;
    }

    /********************************************************************************/
    /**
     * \brief Get amount of possible entries.
     *
     * \return amount of possible entries
     */
    EC_T_DWORD GetSize(EC_T_VOID) const
    {
        if (EC_NULL == m_pDesc)
        {
            return 0;
        }
        return m_pDesc->dwBufSize - 1;
    }

    /********************************************************************************/
    /**
     * \brief States if Fifo is full.
     *
     * \return EC_TRUE if Fifo is full else EC_FALSE
     */
    EC_T_BOOL IsFull() const
    {
        return GetCount() == GetSize();
    }

    /********************************************************************************/
    /**
     * \brief States if Fifo is empty
     *
     * \return EC_TRUE if Fifo is empty else EC_FALSE
     */
    EC_T_BOOL IsEmpty() const
    {
        return GetCount() == 0;
    }

    /********************************************************************************/
    /**
     * \brief Add object. Thread safe in case of providing a OsLock object
     *          when constructing the FIFO.
     *
     * \return EC_TRUE on success
     */
    EC_T_BOOL Add(VALUE newValue)
    {
        EC_T_BOOL bRes = EC_FALSE;
        if (m_poLock != EC_NULL)
        {
            OsLock(m_poLock);
        }
        bRes = AddNoLock(newValue);
        if (m_poLock != EC_NULL)
        {
            OsUnlock(m_poLock);
        }
        return bRes;
    }

    /********************************************************************************/
    /**
     * \brief Add object.
     *
     * \return Add without OsLock. --> Just one "Adding" Thread is allowed!!!
     */
    EC_T_BOOL AddNoLock(VALUE newValue)
    {
        EC_PT_FIFO_DESC pDesc = m_pDesc;

        if ((EC_NULL == pDesc) || (0 == pDesc->dwBufSize))
        {
            OsDbgAssert(EC_FALSE); /* calling InitInstance missing? */
            return EC_FALSE;
        }
        if (GetCount() == GetSize())
        {
            return EC_FALSE;
        }
        m_pFiFoEntrys[pDesc->dwLast] = newValue;
        OsMemoryBarrier();
        pDesc->dwLast = (pDesc->dwLast + 1) % pDesc->dwBufSize;

        return EC_TRUE;
    }


    /********************************************************************************/
    /**
     * \brief Remove element.Thread safe in case of providing a OsLock object
     *        when constructing the FIFO.
     *
     * \return
     */
    EC_T_BOOL Remove( VALUE& rValue )
    {
        EC_T_BOOL bRes = EC_FALSE;

        if (m_poLock != EC_NULL)
        {
            OsLock(m_poLock);
        }
        bRes = RemoveNoLock(rValue);
        if (m_poLock != EC_NULL)
        {
            OsUnlock(m_poLock);
        }
        return bRes;
    }

    /********************************************************************************/
    /**
     * \brief Remove element
     *
     * \return  Removing without OsLock. --> Just one "Removing" Thread is allowed!!!
     */
    EC_T_BOOL RemoveNoLock(VALUE& rValue)
    {
        EC_PT_FIFO_DESC pDesc = m_pDesc;

        if ((EC_NULL == pDesc) || (0 == pDesc->dwBufSize))
        {
            OsDbgAssert(EC_FALSE); /* calling InitInstance missing? */
            return EC_FALSE;
        }
        if (pDesc->dwFirst == pDesc->dwLast)
        {
            return EC_FALSE;
        }
        rValue = m_pFiFoEntrys[pDesc->dwFirst];
        OsMemoryBarrier();
        pDesc->dwFirst = (pDesc->dwFirst + 1) % pDesc->dwBufSize;

        return EC_TRUE;
    }

    EC_T_VOID Clear()
    {
        if (m_poLock != EC_NULL)
        {
            OsLock(m_poLock);
        }
        ClearNoLock();
        if (m_poLock != EC_NULL)
        {
            OsUnlock(m_poLock);
        }
    }

    EC_T_VOID ClearNoLock()
    {
        VALUE Value;
        while (RemoveNoLock(Value)) {}
    }

    /********************************************************************************/
    /**
     * \brief Get next element without remove
     *
     * \return  Peek without OsLock. --> Just one "Peek" Thread is allowed!!!
     */
    EC_T_BOOL PeekNoLock( VALUE& rValue )
    {
        EC_PT_FIFO_DESC pDesc = m_pDesc;

        if ((EC_NULL == pDesc) || (0 == pDesc->dwBufSize))
        {
            return EC_FALSE;
        }
        if (pDesc->dwFirst == pDesc->dwLast)
        {
            return EC_FALSE;
        }
        OsMemoryBarrier();
        rValue = m_pFiFoEntrys[pDesc->dwFirst];

        return EC_TRUE;
    }
};

/*-CFiFoListDyn--------------------------------------------------------------*/
template <class VALUE>
class CFiFoListDyn : public CFiFoList<VALUE>
{
public:
    CFiFoListDyn(EC_T_DWORD dwSize, EC_T_VOID* poLock, const EC_T_CHAR* szName, const struct _EC_T_LOG_PARMS* poLogParms = EC_NULL, EC_T_DWORD dwMemTrace = 0, struct _EC_T_MEMORY_LOGGER* poMemLog = EC_NULL)
        : CFiFoList<VALUE>(EC_NULL, EC_NULL, poLock, szName), m_poLogParms((struct _EC_T_LOG_PARMS*)poLogParms), m_dwMemTrace(dwMemTrace), m_poMemLog(poMemLog)
    {
        EC_UNREFPARM(dwSize);
    }
    CFiFoListDyn(const CFiFoListDyn&) { OsDbgAssert(EC_FALSE); } /* objects are not copied */
    virtual ~CFiFoListDyn()
    {
        FreeInstance();
    }

    /********************************************************************************/
    /**
     * \brief Free memory
     */
    virtual EC_T_VOID FreeInstance(EC_T_VOID)
    {
#if (defined EC_TRACE_SUBMEM) && (defined EC_MASTER) && (defined EC_NAMESPACE)
        if (EC_NULL != CFiFoList<VALUE>::m_pFiFoEntrys)
        {
            EC_TRACE_SUBMEM(m_dwMemTrace, "CFiFoListDyn::~CFiFoListDyn CFiFoList<VALUE>::m_pFiFoEntrys", CFiFoList<VALUE>::m_pFiFoEntrys, CFiFoList<VALUE>::m_pDesc->dwBufSize * sizeof(VALUE)); 
        }
        if (EC_NULL != CFiFoList<VALUE>::m_pDesc)
        {
            EC_TRACE_SUBMEM(m_dwMemTrace, "CFiFoListDyn::~CFiFoListDyn CFiFoList<VALUE>::m_pDesc", CFiFoList<VALUE>::m_pDesc, sizeof(EC_T_FIFO_DESC)); 
        }
#endif

        SafeDeleteArray(CFiFoList<VALUE>::m_pFiFoEntrys);
        OsSafeFree(CFiFoList<VALUE>::m_pDesc);
    }

    /********************************************************************************/
    /**
     * \brief Allocate memory
     *
     * \return #EC_E_NOERROR or error code
     */
    virtual EC_T_DWORD InitInstance(EC_T_DWORD dwSize)
    {
        EC_PT_FIFO_DESC pDesc = EC_NULL;
        VALUE* pFiFoEntrys = EC_NULL;
        EC_T_DWORD dwRetVal = EC_E_NOERROR;

        EC_T_DWORD dwBufSize = dwSize + 1;

        if (EC_NULL != CFiFoList<VALUE>::m_pDesc)
        {
            dwRetVal = EC_E_INVALIDSTATE;
            goto Exit;
        }

        pDesc = (EC_PT_FIFO_DESC)OsMalloc(sizeof(EC_T_FIFO_DESC));
        if (EC_NULL == pDesc)
        {
            dwRetVal = EC_E_NOMEMORY;
            goto Exit;
        }

        pFiFoEntrys = EC_NEW(VALUE[dwBufSize]);
        if (EC_NULL == pFiFoEntrys)
        {
            dwRetVal = EC_E_NOMEMORY;
            goto Exit;
        }

        /* no error */
        CFiFoList<VALUE>::m_pDesc = pDesc;
        CFiFoList<VALUE>::m_pFiFoEntrys = pFiFoEntrys;

#if (defined EC_TRACE_ADDMEM) && (defined EC_MASTER) && (defined EC_NAMESPACE)
        EC_TRACE_ADDMEM(m_dwMemTrace, "CFiFoListDyn::InitInstance m_pDesc", pDesc, sizeof(EC_T_FIFO_DESC)); 
        EC_TRACE_ADDMEM(m_dwMemTrace, "CFiFoListDyn::InitInstance m_pFiFoEntrys", pFiFoEntrys, dwBufSize * sizeof(VALUE)); 
#endif

        OsMemset(CFiFoList<VALUE>::m_pDesc, 0, sizeof(EC_T_FIFO_DESC));
        OsMemset(CFiFoList<VALUE>::m_pFiFoEntrys, 0, dwBufSize * sizeof(VALUE));

        CFiFoList<VALUE>::m_pDesc->dwBufSize = dwBufSize;

        dwRetVal = EC_E_NOERROR;
    Exit:
        if (EC_E_NOERROR != dwRetVal)
        {
            SafeDeleteArray(pFiFoEntrys);
            OsSafeFree(pDesc);
        }

        return dwRetVal;
    }

    /* Logging */
    struct _EC_T_LOG_PARMS*     m_poLogParms;
    EC_INLINESTART struct _EC_T_LOG_PARMS* GetLogParms() { return m_poLogParms; } EC_INLINESTOP

    /* Memory Logger */
    EC_T_DWORD                  m_dwMemTrace;
    struct _EC_T_MEMORY_LOGGER* m_poMemLog;
    EC_INLINESTART struct _EC_T_MEMORY_LOGGER* GetMemLog() { return m_poMemLog; } EC_INLINESTOP
};

#endif /* INC_ECFIFO */

/*-END OF SOURCE FILE--------------------------------------------------------*/
