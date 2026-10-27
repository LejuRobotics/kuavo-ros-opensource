/*-----------------------------------------------------------------------------
 * EcFiFo.h                 header file
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Stefan Zintgraf
 * Description
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
    VALUE*          m_pFiFoEntrys;
    volatile EC_PT_FIFO_DESC m_pDesc;
    EC_T_VOID*      m_poLock;
#ifdef DEBUGTRACE
    EC_T_DWORD      m_dwMemTrace;
#endif

public:
    CFiFoList(EC_PT_FIFO_DESC pDesc, VALUE* pFiFoEntrys, EC_T_VOID* poLock, const EC_T_CHAR* szName, EC_T_DWORD dwMemTrace = 0)
    {
        EC_UNREFPARM(szName);
        m_pDesc         = pDesc;
        m_pFiFoEntrys   = pFiFoEntrys;
        m_poLock        = poLock;
#ifdef DEBUGTRACE
        m_dwMemTrace    = dwMemTrace;
#else
        EC_UNREFPARM(dwMemTrace);
#endif
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
     * \return  Peak without OsLock. --> Just one "Peak" Thread is allowed!!!
     */
    EC_T_BOOL PeakNoLock( VALUE& rValue )
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
        rValue = m_pFiFoEntrys[pDesc->dwFirst];

        return EC_TRUE;
    }
};

/*-CFiFoListDyn--------------------------------------------------------------*/
template <class VALUE>
class CFiFoListDyn : public CFiFoList<VALUE>
{
public:

    CFiFoListDyn(EC_T_DWORD dwSize, EC_T_VOID* poLock, const EC_T_CHAR* szName, EC_T_DWORD dwMemTrace = 0)
        : CFiFoList<VALUE>(EC_NULL, EC_NULL, poLock, szName, dwMemTrace)
    {
        EC_UNREFPARM(dwSize);
    }
    CFiFoListDyn(const CFiFoListDyn&) { OsDbgAssert(EC_FALSE); } /* objects are not copied */
    virtual ~CFiFoListDyn()
    {
        SafeDeleteArray(CFiFoList<VALUE>::m_pFiFoEntrys);
        SafeOsFree(CFiFoList<VALUE>::m_pDesc);
    }

    /********************************************************************************/
    /**
     * \brief initialize the instance.
     *
     * \return
     */
    virtual EC_T_DWORD InitInstance(EC_T_DWORD dwSize) // TODO: EC_TRACE_ADDMEM, EC_TRACE_SUBMEM
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

        CFiFoList<VALUE>::m_pDesc = pDesc;
        CFiFoList<VALUE>::m_pFiFoEntrys = pFiFoEntrys;

        OsMemset(CFiFoList<VALUE>::m_pDesc, 0, sizeof(EC_T_FIFO_DESC));
        OsMemset(CFiFoList<VALUE>::m_pFiFoEntrys, 0, dwBufSize * sizeof(VALUE));

        CFiFoList<VALUE>::m_pDesc->dwBufSize = dwBufSize;

        dwRetVal = EC_E_NOERROR;
    Exit:
        if (EC_E_NOERROR != dwRetVal)
        {
            SafeDeleteArray(pFiFoEntrys);
            SafeOsFree(pDesc);
        }

        return dwRetVal;
    }
};

#endif /* INC_ECFIFO */

/*-END OF SOURCE FILE--------------------------------------------------------*/
